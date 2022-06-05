#include "usv_map/skeleton_generator.h"

VoronoiSkeletonGenerator::VoronoiSkeletonGenerator(std::string layername, OGRPoint& lower_left, OGRPoint& upper_right, GDALDataset* ds, std::string mission_region, Quadtree* tree, MapService* map_service, GeographicLib::Geodesic* geod):
ds_(ds),
mission_region_(mission_region),
tree_(tree),
map_service_(map_service),
geod_(geod),
lower_left_(lower_left),
upper_right_(upper_right){
    in_layer_ = ds_->GetLayerByName(layername.c_str());
    voronoi_layer_ = ds_->CreateLayer("voronoi",nullptr,wkbMultiLineString);
}

/**
 * @brief Run the Voronoi SKeleton Generator.
 * 
 */
void VoronoiSkeletonGenerator::run(){
    ros::Time start = ros::Time::now();
    OGRFeature* feat;
    std::vector<jcv_point> points_vec;
    jcv_point new_point;
    OGRLinearRing* ring;

    in_layer_->ResetReading();
    while((feat = in_layer_->GetNextFeature()) != NULL){
        ring = feat->GetGeometryRef()->toPolygon()->getExteriorRing();
        int num_points = ring->getNumPoints();
        for(int i=0; i<num_points-1;i++){
            new_point.x = ring->getX(i);
            new_point.y = ring->getY(i);
            points_vec.push_back(new_point);
        }
        OGRFeature::DestroyFeature(feat);
    }
    sites_count_ = points_vec.size();

    jcv_diagram diagram;
    memset(&diagram, 0, sizeof(jcv_diagram));
    jcv_diagram_generate(points_vec.size(), points_vec.data(), 0, 0, &diagram);
    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);

    buildSkeleton(diagram);
    jcv_diagram_free( &diagram );
    total_build_time_ = ros::Duration(ros::Time::now()-start).toSec();
    dumpDebug();
}

/**
 * @brief Procedure for taking in a Voronoi Diagram of a mission region
 * and building a Voronoi skeleton from it.
 * 
 * @param diagram 
 */
void VoronoiSkeletonGenerator::buildSkeleton(jcv_diagram& diagram){
    //Identify for every point every edge connected to it
    ros::Time start = ros::Time::now();
    std::unordered_map<int,const jcv_edge*> edge_map;
    boost::unordered_map<std::pair<int,int>,std::vector<int64_t>> point_map;

    //Register candidate edges from GVD. Edges colliding with geometry or outside region are not considered candiates.
    std::cout << "Register candidate edges" << std::endl;
    registerCandidateEdges(diagram, edge_map, point_map);

    //Identify prune candidates and remove them. This is the main algorithm in the skeleton generator.
    std::cout << "pruneEdges" << std::endl;
    pruneEdges(edge_map,point_map);

    //From point map identify all remaining unique edges
    std::cout << "identifyUniqueEdges" << std::endl;
    std::set<const jcv_edge*> unique_remaining_edges;
    identifyUniqueEdges(unique_remaining_edges,edge_map,point_map);

    //Add unique edges to dataset voronoi layer.
    std::cout << "addEdgesToDataset" << std::endl;
    addEdgesToDataset(unique_remaining_edges);
    benchmark_data_.build_time = ros::Duration(ros::Time::now()-start).toSec();
}

/**
 * @brief Go throug all edges in Voronoi diagram and remove all that 
 * can be trivially discarded due to land intersection or mission region border
 * intersection
 * 
 * @param diagram 
 * @param edge_map 
 * @param point_map 
 */
void VoronoiSkeletonGenerator::registerCandidateEdges(jcv_diagram& diagram, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    const jcv_edge* edges_for_counting = jcv_diagram_get_edges(&diagram);
    int counter = 0;
    while(edges_for_counting){
        counter++;
        edges_for_counting = jcv_diagram_get_next_edge(edges_for_counting);
    }
    benchmark_data_.original_edges = counter;

    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);
    int edge_id = 0;
    std::pair<double,double> tmp_point_pair;
    diagram_edges_count_=0;
    ros::Time start;
    while(edges){
        start = ros::Time::now();
        diagram_edges_count_++;
        if(diagram_edges_count_%1000==0){
            std::cout << "Processed edges: " << diagram_edges_count_ << std::endl;
        }
        //First check if edge outside region, if so discard edge 
        if(!pointInRegion(edges->pos[0].x,edges->pos[0].y)||!pointInRegion(edges->pos[1].x,edges->pos[1].y)){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }

        //First check for completely invalid edges that atually collide with land and remove them
        if(collision(edges)){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }

        int id = edge_id++;
        edge_map[id] = edges;
        
        //Add edge to point map
        tmp_point_pair.first = edges->pos[0].x*precision;
        tmp_point_pair.second = edges->pos[0].y*precision;
        point_map[tmp_point_pair].push_back(id);
        tmp_point_pair.first = edges->pos[1].x*precision;
        tmp_point_pair.second = edges->pos[1].y*precision;
        point_map[tmp_point_pair].push_back(id);

        edges = jcv_diagram_get_next_edge(edges);
        benchmark_data_.candidateEvaluation_time.push_back(ros::Duration(ros::Time::now()-start).toSec());
    }

    benchmark_data_.candidate_edges = edge_map.size();
}

/**
 * @brief Itertaively prune all edges in an edge map to remove edges not belonging in Voronoi skeleton but not trivially
 * discarded by intersection checking.
 * 
 * @param edge_map 
 * @param point_map 
 */
void VoronoiSkeletonGenerator::pruneEdges(std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    ros::Time start;
    ros::Time start_sub;
    std::pair<double,double> tmp_point_pair;
    std::map<std::pair<double,double>,double> distance_map;

    int iteration_count=0;
    while(true){
        start = ros::Time::now();
        std::vector<const jcv_edge*> prune_candidates;
        for (auto point_map_it = point_map.begin(); point_map_it!=point_map.end(); point_map_it++){
            if (point_map_it->second.size()==1){
                prune_candidates.push_back(edge_map[point_map_it->second.at(0)]);
            }
        }
        int pruned = 0;
        for(auto prune_candidate_it=prune_candidates.begin(); prune_candidate_it!=prune_candidates.end(); prune_candidate_it++ ){
            start_sub = ros::Time::now();
            //Try to prune branches that are approximately normal to land
            double edge_length;
            geod_->Inverse((*prune_candidate_it)->pos[0].y,(*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[1].y,(*prune_candidate_it)->pos[1].x,edge_length);
            edge_length=abs(edge_length);
            
            double d0=0;
            double d1=0;
            auto d0_map_it = distance_map.find(std::make_pair((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y));
            auto d1_map_it = distance_map.find(std::make_pair((*prune_candidate_it)->pos[1].x,(*prune_candidate_it)->pos[1].y));
            if(d0_map_it==distance_map.end()){
                //ROS_WARN_COND(iteration_count!=0, "Should not be here, should use distance map");
                d0=map_service_->distance((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y,LayerID::COLLISION,INFINITY);
                distance_map.insert(std::make_pair(std::make_pair((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y),d0));
            } else{
                d0 = (*d0_map_it).second;
            }
            if(d1_map_it==distance_map.end()){
                //ROS_WARN_COND(iteration_count!=0, "Should not be here, should use distance map");
                d1=map_service_->distance((*prune_candidate_it)->pos[1].x,(*prune_candidate_it)->pos[1].y,LayerID::COLLISION,INFINITY);
                distance_map.insert(std::make_pair(std::make_pair((*prune_candidate_it)->pos[1].x,(*prune_candidate_it)->pos[1].y),d1));
            } else{
                d1 = (*d1_map_it).second;
            }
            double diff = abs(abs(d1)-abs(d0))*1e5;
            
            
            if(abs(edge_length/diff)<=1.5){
                pruned++;
                tmp_point_pair.first = (*prune_candidate_it)->pos[0].x*precision;
                tmp_point_pair.second = (*prune_candidate_it)->pos[0].y*precision;
                for(auto it = point_map[tmp_point_pair].begin(); it!= point_map[tmp_point_pair].end(); it++){
                    if(edge_map[*it]==(*prune_candidate_it)){
                        point_map[tmp_point_pair].erase(it);
                        break;
                    }
                }
                tmp_point_pair.first = (*prune_candidate_it)->pos[1].x*precision;
                tmp_point_pair.second = (*prune_candidate_it)->pos[1].y*precision;
                for(auto it = point_map[tmp_point_pair].begin(); it!= point_map[tmp_point_pair].end(); it++){
                    if(edge_map[*it]==(*prune_candidate_it)){
                        point_map[tmp_point_pair].erase(it);
                        break;
                    }
                }
            }
            benchmark_data_.pruneEdgeEvaluation_time.push_back(ros::Duration(ros::Time::now()-start_sub).toSec());
        }
        //std::cout << "Pruned: " << pruned << std::endl;
        pruned_edges_count_+=pruned;
        benchmark_data_.pruneIteration_time.push_back(ros::Duration(ros::Time::now()-start).toSec());
        if (pruned==0){
            break;
        }

        if(!ros::ok()){
            exit(1);
        }
        iteration_count++;
    }
    benchmark_data_.skeleton_edges = pruned_edges_count_;
}

/**
 * @brief Reconstruct edge map from point map
 * 
 * @param unique_remaining_edges 
 * @param edge_map 
 * @param point_map 
 */
void VoronoiSkeletonGenerator::identifyUniqueEdges(std::set<const jcv_edge*>& unique_remaining_edges,std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    ros::Time start = ros::Time::now();
    for (auto point_map_it = point_map.begin(); point_map_it!=point_map.end(); point_map_it++){
        for(auto edge_it = point_map_it->second.begin(); edge_it!=point_map_it->second.end(); edge_it++){
            unique_remaining_edges.insert(edge_map[*edge_it]);
        }
    }
    skeleton_edges_count_ = unique_remaining_edges.size();
    identify_unique_edges_time_=ros::Duration(ros::Time::now()-start).toSec();
}

/**
 * @brief Add Voronoi skeleton edges to mission region spatial database.
 * 
 * @param unique_remaining_edges 
 */
void VoronoiSkeletonGenerator::addEdgesToDataset(std::set<const jcv_edge*>& unique_remaining_edges){
    ros::Time start = ros::Time::now();
    OGRPoint point_a, point_b;
    OGRFeature* voronoi_feature = OGRFeature::CreateFeature(voronoi_layer_->GetLayerDefn());
    voronoi_feature->SetFID(0);
    OGRMultiLineString voronoi_geom;

    for(auto it = unique_remaining_edges.begin(); it!= unique_remaining_edges.end(); it++){
        //Add unique edges
        point_a.setX((*it)->pos[0].x);
        point_a.setY((*it)->pos[0].y);
        point_b.setX((*it)->pos[1].x);
        point_b.setY((*it)->pos[1].y);
        OGRLineString line;
        line.addPoint(&point_a);
        line.addPoint(&point_b);
        voronoi_geom.addGeometry(&line);
    }

    voronoi_feature->SetGeometry(&voronoi_geom);
    voronoi_layer_->CreateFeature(voronoi_feature);
    add_edges_to_dataset_time_ = ros::Duration(ros::Time::now()-start).toSec();
}

/**
 * @brief Supporting function to check if a geodetic position (lon,lat) is
 * within the mission region extent.
 * 
 * @param lon 
 * @param lat 
 * @return true 
 * @return false 
 */
bool VoronoiSkeletonGenerator::pointInRegion(double lon, double lat){
    ros::Time start = ros::Time::now();
    bool in_region = lon>=lower_left_.getX() && lat>=lower_left_.getY() && lon<=upper_right_.getX() && lat<=upper_right_.getY();
    point_in_region_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return in_region;
}

/**
 * @brief Supporting function to check if a Voronoi Diagram edge
 * collides with any geometry in the collision layer of the mission region overview 
 * spatial database.
 * 
 * @param edge 
 * @return true 
 * @return false 
 */
bool VoronoiSkeletonGenerator::collision(const jcv_edge* edge){
    ros::Time start = ros::Time::now();
    bool intersects = false;

    OGRPoint point_a, point_b;
    point_a.setX(edge->pos[0].x);
    point_a.setY(edge->pos[0].y);
    point_b.setX(edge->pos[1].x);
    point_b.setY(edge->pos[1].y);
    OGRLineString line;
    line.addPoint(&point_a);
    line.addPoint(&point_b);
    intersects = map_service_->intersects(&line,LayerID::COLLISION);
    collision_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return intersects;
}

void VoronoiSkeletonGenerator::dumpDebug(){
    std::string path = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_+"/benchmark/voronoiSkeleton/";
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directories(path);
    }

    std::ofstream benchmark_file_time(path+"benchmark_time.csv");
    std::ofstream benchmark_file_misc(path+"benchmark_misc.csv");
    benchmark_file_time<<"name,times_run,total_time\n";
    benchmark_file_misc<<"name,value\n";

    ROS_INFO_STREAM("Voronoi skeleton time spent [s] debug data: ");
    ROS_INFO_STREAM("Overall build: " << benchmark_data_.build_time);
    benchmark_file_misc<<"original_edges"<<","<< benchmark_data_.original_edges << "\n";
    benchmark_file_misc<<"candidate_edges"<<","<< benchmark_data_.candidate_edges << "\n";
    benchmark_file_misc<<"skeleton_edges"<<","<<1<<","<<benchmark_data_.skeleton_edges<<"\n";
    benchmark_file_time<<"candidate_evaluation"<<","<<benchmark_data_.candidateEvaluation_time.size()<<","<<std::accumulate(benchmark_data_.candidateEvaluation_time.begin(),benchmark_data_.candidateEvaluation_time.end(),0.0)<<"\n";
    benchmark_file_time<<"prune_iteration"<<","<<benchmark_data_.pruneIteration_time.size()<<","<<std::accumulate(benchmark_data_.pruneIteration_time.begin(),benchmark_data_.pruneIteration_time.end(),0.0)<<"\n";
    benchmark_file_time<<"pruneEdgeEvaluation_time"<<","<<benchmark_data_.pruneEdgeEvaluation_time.size()<<","<<std::accumulate(benchmark_data_.pruneEdgeEvaluation_time.begin(),benchmark_data_.pruneEdgeEvaluation_time.end(),0.0)<<"\n";

    ROS_INFO_STREAM("Checking for collision: " << collision_times_.size() << " times. Total time spent: " << std::accumulate(collision_times_.begin(),collision_times_.end(),0.0));
    ROS_INFO_STREAM("Checking for point in region: " << point_in_region_times_.size() << " times. Total time spent: " << std::accumulate(point_in_region_times_.begin(),point_in_region_times_.end(),0.0));
    ROS_INFO_STREAM("");
    ROS_INFO_STREAM("Misc. numbers: ");
    ROS_INFO_STREAM("Sites count: " << sites_count_);
    ROS_INFO_STREAM("Original Voronoi diagram edges: " << benchmark_data_.original_edges);
    ROS_INFO_STREAM("Candidate edges count: " << benchmark_data_.candidate_edges);
    ROS_INFO_STREAM("Skeleton edges count: " << benchmark_data_.skeleton_edges);
}