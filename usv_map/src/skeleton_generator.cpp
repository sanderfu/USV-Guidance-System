#include "usv_map/skeleton_generator.h"

VoronoiSkeletonGenerator::VoronoiSkeletonGenerator(std::string layername, OGRPoint& lower_left, OGRPoint& upper_right, GDALDataset* ds, Quadtree* tree, MapService* map_service, GeographicLib::Geodesic* geod):
ds_(ds),
tree_(tree),
map_service_(map_service),
geod_(geod),
lower_left_(lower_left),
upper_right_(upper_right){
    in_layer_ = ds_->GetLayerByName(layername.c_str());
    voronoi_layer_ = ds_->CreateLayer("voronoi",nullptr,wkbMultiLineString);
}

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
    build_skeleton_time_ = ros::Duration(ros::Time::now()-start).toSec();
}

void VoronoiSkeletonGenerator::registerCandidateEdges(jcv_diagram& diagram, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    ros::Time start = ros::Time::now();

    const jcv_edge* edges_for_counting = jcv_diagram_get_edges(&diagram);
    int counter = 0;
    while(edges_for_counting){
        counter++;
        edges_for_counting = jcv_diagram_get_next_edge(edges_for_counting);
    }
    std::cout << "Total amount of edges: " << counter << std::endl;

    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);
    int edge_id = 0;
    std::pair<double,double> tmp_point_pair;
    diagram_edges_count_=0;
    while(edges){
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
    }
    register_candidate_edges_time_ = ros::Duration(ros::Time::now()-start).toSec();
}

void VoronoiSkeletonGenerator::pruneEdges(std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    ros::Time start = ros::Time::now();
    std::pair<double,double> tmp_point_pair;
    std::map<std::pair<double,double>,double> distance_map;
    while(true){
        std::vector<const jcv_edge*> prune_candidates;
        for (auto point_map_it = point_map.begin(); point_map_it!=point_map.end(); point_map_it++){
            if (point_map_it->second.size()==1){
                prune_candidates.push_back(edge_map[point_map_it->second.at(0)]);
            }
        }
        //std::cout << "Prune candidates: " << prune_candidates.size() << std::endl;
        int pruned = 0;
        for(auto prune_candidate_it=prune_candidates.begin(); prune_candidate_it!=prune_candidates.end(); prune_candidate_it++ ){
            //Try to prune branches that are approximately normal to land
            double edge_length;
            geod_->Inverse((*prune_candidate_it)->pos[0].y,(*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[1].y,(*prune_candidate_it)->pos[1].x,edge_length);
            edge_length=abs(edge_length);
            
            double d0=0;
            double d1=0;
            auto d0_map_it = distance_map.find(std::make_pair((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y));
            auto d1_map_it = distance_map.find(std::make_pair((*prune_candidate_it)->pos[1].x,(*prune_candidate_it)->pos[1].y));
            if(d0_map_it==distance_map.end()){
                d0=map_service_->distance((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y,LayerID::COLLISION,INFINITY);
                distance_map.insert(std::make_pair(std::make_pair((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y),d0));
            } else{
                d0 = (*d0_map_it).second;
            }
            if(d1_map_it==distance_map.end()){
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
        }
        //std::cout << "Pruned: " << pruned << std::endl;
        pruned_edges_count_+=pruned;
        if (pruned==0){
            break;
        }

        if(!ros::ok()){
            exit(1);
        }
    }
    prune_edges_time_ = ros::Duration(ros::Time::now()-start).toSec();
}

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

bool VoronoiSkeletonGenerator::pointInRegion(double lon, double lat){
    ros::Time start = ros::Time::now();
    bool in_region = lon>=lower_left_.getX() && lat>=lower_left_.getY() && lon<=upper_right_.getX() && lat<=upper_right_.getY();
    point_in_region_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return in_region;
}

bool VoronoiSkeletonGenerator::collision(const jcv_edge* edge){
    ros::Time start = ros::Time::now();
    bool intersects = false;
    
    //Fast method but relies on quadtree minimum region area, so disabled (at least for now)
    //if(tree_->getLeafRegionContaining(edge->pos[0].x,edge->pos[0].y)==nullptr || tree_->getLeafRegionContaining(edge->pos[1].x,edge->pos[1].y)==nullptr){
    //    intersects=true;
    //}

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

double VoronoiSkeletonGenerator::smallestDistanceMeasured(boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    double dist = 0;
    double smallest_dist = INFINITY;
    for(auto point_a_it=point_map.begin(); point_a_it!=point_map.end(); point_a_it++){
        for(auto point_b_it=point_map.begin(); point_b_it!=point_map.end(); point_b_it++){
            if(point_a_it!=point_b_it){
                GeographicLib::Geodesic::WGS84().Inverse(point_a_it->first.second/precision,point_a_it->first.first/precision,point_b_it->first.second/precision,point_b_it->first.first/precision,dist);
                dist = abs(dist);
                if(dist<smallest_dist){
                    smallest_dist=dist;
                }
            }
        }
    }
}

void VoronoiSkeletonGenerator::dumpDebug(){
    ROS_INFO_STREAM("Voronoi skeleton time spent [s] debug data: ");
    ROS_INFO_STREAM("Overall build: " << total_build_time_);
    ROS_INFO_STREAM("-Build skeleton: " << build_skeleton_time_);
    ROS_INFO_STREAM("--Register candidate edges: " << register_candidate_edges_time_);
    ROS_INFO_STREAM("--Prune edges: " << prune_edges_time_);
    ROS_INFO_STREAM("--Identify unique edges: " << identify_unique_edges_time_);
    ROS_INFO_STREAM("--Add edges to dataset: " << add_edges_to_dataset_time_);

    ROS_INFO_STREAM("Checking for collision: " << collision_times_.size() << " times. Total time spent: " << std::accumulate(collision_times_.begin(),collision_times_.end(),0.0));
    ROS_INFO_STREAM("Checking for point in region: " << point_in_region_times_.size() << " times. Total time spent: " << std::accumulate(point_in_region_times_.begin(),point_in_region_times_.end(),0.0));
    ROS_INFO_STREAM("");
    ROS_INFO_STREAM("Misc. numbers: ");
    ROS_INFO_STREAM("Sites count: " << sites_count_);
    ROS_INFO_STREAM("Original Voronoi diagram edges: " << diagram_edges_count_);
    ROS_INFO_STREAM("Pruned edges count: " << pruned_edges_count_);
    ROS_INFO_STREAM("Skeleton edges count: " << skeleton_edges_count_);
}