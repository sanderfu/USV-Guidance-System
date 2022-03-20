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
    OGRFeature* feat;
    std::vector<jcv_point> points_vec;
    OGRPoint* point;
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

    jcv_diagram diagram;
    memset(&diagram, 0, sizeof(jcv_diagram));
    jcv_diagram_generate(points_vec.size(), points_vec.data(), 0, 0, &diagram);
    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);

    OGRPoint check_point_a;
    OGRPoint check_point_b;
    OGRGeometry* geom;
    int total_edges = 0;
    int fast_check_edges = 0;
    int branches_pruned = 0;

    //std::unordered_map<std::pair<double,double>,bool,hash_pair> contained_in_geometry_lookup;
    pruneEdges(diagram);
    jcv_diagram_free( &diagram );
}

void VoronoiSkeletonGenerator::pruneEdges(jcv_diagram& diagram){
    //Identify for every point every edge connected to it
    std::unordered_map<int,const jcv_edge*> edge_map;
    boost::unordered_map<std::pair<int,int>,std::vector<int64_t>> point_map;

    //Register candidate edges from GVD. Edges colliding with geometry or outside region are not considered candiates.
    registerCandidateEdges(diagram, edge_map, point_map);

    //Check if any key points are super close (for debug purposes)
    //std::cout << "Smallest distance measured:" << smallestDistanceMeasured(point_map) << std::endl;

    //Identify prune candidates and remove them
    removeInvalidEdges(edge_map,point_map);

    std::set<const jcv_edge*> unique_remaining_edges;
    identifyUniqueEdges(unique_remaining_edges,edge_map,point_map);

    addEdgesToDataset(unique_remaining_edges);
}

void VoronoiSkeletonGenerator::registerCandidateEdges(jcv_diagram& diagram, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);
    int edge_id = 0;
    std::pair<double,double> tmp_point_pair;
    int edge_counter = 0;

    OGRPoint check_point_a;
    OGRPoint check_point_b;
    while(edges){

        //First check if edge outside region, if so discard edge 
        if(!pointInRegion(edges->pos[0].x,edges->pos[0].y)||!pointInRegion(edges->pos[1].x,edges->pos[1].y)){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }

        //First check for completely invalid edges that atually collide with land and remove them
        /*
        if(tree_->getLeafRegionContaining(edges->pos[0].x,edges->pos[0].y)==nullptr || tree_->getLeafRegionContaining(edges->pos[1].x,edges->pos[1].y)==nullptr){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }
        */
        check_point_a.setX(edges->pos[0].x);
        check_point_a.setY(edges->pos[0].y);
        check_point_b.setX(edges->pos[1].x);
        check_point_b.setY(edges->pos[1].y);
        OGRLineString line;
        line.addPoint(&check_point_a);
        line.addPoint(&check_point_b);
        if(map_service_->intersects(&line,LayerID::COLLISION)){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }

        edge_counter++;
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
}

void VoronoiSkeletonGenerator::removeInvalidEdges(std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    std::pair<double,double> tmp_point_pair;
    while(true){
        std::vector<const jcv_edge*> prune_candidates;
        for (auto point_map_it = point_map.begin(); point_map_it!=point_map.end(); point_map_it++){
            if (point_map_it->second.size()==1){
                prune_candidates.push_back(edge_map[point_map_it->second.at(0)]);
            }
        }
        std::cout << "Prune candidates: " << prune_candidates.size() << std::endl;
        int pruned = 0;
        for(auto prune_candidate_it=prune_candidates.begin(); prune_candidate_it!=prune_candidates.end(); prune_candidate_it++ ){
            //Try to prune branches that are approximately normal to land
            double edge_length;
            geod_->Inverse((*prune_candidate_it)->pos[0].y,(*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[1].y,(*prune_candidate_it)->pos[1].x,edge_length);
            edge_length=abs(edge_length);

            double d0=map_service_->distance((*prune_candidate_it)->pos[0].x,(*prune_candidate_it)->pos[0].y,LayerID::COLLISION,INFINITY);
            double d1=map_service_->distance((*prune_candidate_it)->pos[1].x,(*prune_candidate_it)->pos[1].y,LayerID::COLLISION,INFINITY);
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
        std::cout << "Pruned: " << pruned << std::endl;
        if (pruned==0){
            break;
        }

        if(!ros::ok()){
            exit(1);
        }
    }
}

void VoronoiSkeletonGenerator::identifyUniqueEdges(std::set<const jcv_edge*>& unique_remaining_edges,std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map){
    for (auto point_map_it = point_map.begin(); point_map_it!=point_map.end(); point_map_it++){
        for(auto edge_it = point_map_it->second.begin(); edge_it!=point_map_it->second.end(); edge_it++){
            unique_remaining_edges.insert(edge_map[*edge_it]);
        }
    }
}

void VoronoiSkeletonGenerator::addEdgesToDataset(std::set<const jcv_edge*>& unique_remaining_edges){
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
}

bool VoronoiSkeletonGenerator::pointInRegion(double lon, double lat){
    ros::Time start = ros::Time::now();
    return lon>=lower_left_.getX() && lat>=lower_left_.getY() && lon<=upper_right_.getX() && lat<=upper_right_.getY();
    point_in_region_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
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

