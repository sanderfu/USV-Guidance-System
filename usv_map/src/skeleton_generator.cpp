#include "usv_map/skeleton_generator.h"

VoronoiSkeletonGenerator::VoronoiSkeletonGenerator(std::string layername, GDALDataset* ds, Quadtree* tree, MapService* map_service, GeographicLib::Geodesic* geod):
ds_(ds),
tree_(tree),
map_service_(map_service),
geod_(geod){
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

    /*
    OGRFeature* voronoi_feature = OGRFeature::CreateFeature(voronoi_layer_->GetLayerDefn());
    voronoi_feature->SetFID(0);
    OGRMultiLineString voronoi_geom;
    */

    OGRPoint check_point_a;
    OGRPoint check_point_b;
    OGRGeometry* geom;
    int total_edges = 0;
    int fast_check_edges = 0;
    int branches_pruned = 0;

    //std::unordered_map<std::pair<double,double>,bool,hash_pair> contained_in_geometry_lookup;
    pruneEdges(diagram);
    jcv_diagram_free( &diagram );
    return;
    while(edges){
        bool intersects = false;
        total_edges++;
        if(total_edges%1000==0){
            std::cout << "Total edges processed: " << total_edges << std::endl;
        }
        
        if(tree_->getLeafRegionContaining(edges->pos[0].x,edges->pos[0].y)==nullptr || tree_->getLeafRegionContaining(edges->pos[1].x,edges->pos[1].y)==nullptr){
            fast_check_edges++;
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }        
        
        //Try to prune branches that are approximately normal to land
        double edge_length;
        geod_->Inverse(edges->pos[0].y,edges->pos[0].x,edges->pos[1].y,edges->pos[1].x,edge_length);
        edge_length=abs(edge_length);

        double d0=map_service_->distance(edges->pos[0].x,edges->pos[0].y,LayerID::COLLISION,INFINITY);
        double d1=map_service_->distance(edges->pos[1].x,edges->pos[1].y,LayerID::COLLISION,INFINITY);
        double diff = abs(abs(d1)-abs(d0))*1e5;
        //std::cout << abs(edge_length/diff) << std::endl;
        
        
        if(abs(edge_length/diff)<=1.1){
            branches_pruned++;
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }
        
        
        

        //std::cout << "Edge length: " << edge_length << " Diff: " << diff*1e5 << " Difference: " << abs(edge_length-diff) << std::endl;
        
        /*
        auto contained_in_geometry_a = contained_in_geometry_lookup.find(std::make_pair(edges->pos[0].x,edges->pos[0].y));
        auto contained_in_geometry_b = contained_in_geometry_lookup.find(std::make_pair(edges->pos[1].x,edges->pos[1].y));

        if(contained_in_geometry_a!=contained_in_geometry_lookup.end() || contained_in_geometry_b!=contained_in_geometry_lookup.end()){
            //One of the points are registered to be in land geometry, can skip check and set intersects=true
            intersects=true;
        } 
        */
       
       
        check_point_a.setX(edges->pos[0].x);
        check_point_a.setY(edges->pos[0].y);
        check_point_b.setX(edges->pos[1].x);
        check_point_b.setY(edges->pos[1].y);

        in_layer_->ResetReading();
        while((feat = in_layer_->GetNextFeature()) != NULL){
            geom = feat->GetGeometryRef();
            bool check_point_a_intersects = geom->Contains(&check_point_a);
            bool check_point_b_intersects = geom->Contains(&check_point_b);
            /*
            if(check_point_a_intersects){
                contained_in_geometry_lookup.insert(std::make_pair(std::make_pair(edges->pos[0].x,edges->pos[0].y),true));
                intersects=true;
            }
            if(check_point_b_intersects){
                contained_in_geometry_lookup.insert(std::make_pair(std::make_pair(edges->pos[1].x,edges->pos[1].y),true));
                intersects=true;
            }
            */
            if(intersects) break;
            OGRFeature::DestroyFeature(feat);
        }
        
        if (intersects){
            edges = jcv_diagram_get_next_edge(edges);
            continue;
        }
        
        
        //If comes this far, add
        /*OGRLineString line;
        line.addPoint(&check_point_a);
        line.addPoint(&check_point_b);
        voronoi_geom.addGeometry(&line);
        */

        //i++;
        //edge_file << edges->pos[0].x << "," << edges->pos[0].y << "," << edges->pos[1].x << "," << edges->pos[1].y << "\n";
        edges = jcv_diagram_get_next_edge(edges);
    }
    //voronoi_feature->SetGeometry(&voronoi_geom);
    //voronoi_layer_->CreateFeature(voronoi_feature);
    jcv_diagram_free( &diagram );
}

void VoronoiSkeletonGenerator::pruneEdges(jcv_diagram diagram){
    //Identify for every point every edge connected to it
    std::unordered_map<int,const jcv_edge*> edge_map;
    boost::unordered_map<std::pair<int,int>,std::vector<int64_t>> point_map;
    const jcv_edge* edges = jcv_diagram_get_edges(&diagram);
    int edge_id = 0;
    std::pair<double,double> tmp_point_pair;
    int edge_counter = 0;

    int precision = 1e6;
    while(edges){

        //First check for completely invalid edges that atually collide with land and remove them
        if(tree_->getLeafRegionContaining(edges->pos[0].x,edges->pos[0].y)==nullptr || tree_->getLeafRegionContaining(edges->pos[1].x,edges->pos[1].y)==nullptr){
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
        std::cout << edges->pos[0].x << " " << edges->pos[0].y << std::endl;
        std::cout << tmp_point_pair.first << " " << tmp_point_pair.second << std::endl;
        tmp_point_pair.first = edges->pos[1].x*precision;
        tmp_point_pair.second = edges->pos[1].y*precision;
        point_map[tmp_point_pair].push_back(id);

        edges = jcv_diagram_get_next_edge(edges);
    }

    //Check if any points are super close (less than 1 cm)
    std::cout << "Checking if any point is super close to other point" << std::endl;
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
    std::cout << "Smallest distance measured:" << smallest_dist << std::endl;

    //Identify prune candidates
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

    std::set<const jcv_edge*> unique_remaining_edges;
    for (auto point_map_it = point_map.begin(); point_map_it!=point_map.end(); point_map_it++){
        for(auto edge_it = point_map_it->second.begin(); edge_it!=point_map_it->second.end(); edge_it++){
            unique_remaining_edges.insert(edge_map[*edge_it]);
        }
    }
    std::cout << "Edges before pruning: " << edge_counter << std::endl;
    std::cout << "Edges after pruning: " << unique_remaining_edges.size() << std::endl;

    OGRFeature* voronoi_feature = OGRFeature::CreateFeature(voronoi_layer_->GetLayerDefn());
    voronoi_feature->SetFID(0);
    OGRMultiLineString voronoi_geom;
    OGRPoint check_point_a;
    OGRPoint check_point_b;



    for(auto it = unique_remaining_edges.begin(); it!= unique_remaining_edges.end(); it++){
        //Add unique edges
        check_point_a.setX((*it)->pos[0].x);
        check_point_a.setY((*it)->pos[0].y);
        check_point_b.setX((*it)->pos[1].x);
        check_point_b.setY((*it)->pos[1].y);
        OGRLineString line;
        line.addPoint(&check_point_a);
        line.addPoint(&check_point_b);
        voronoi_geom.addGeometry(&line);
    }

    voronoi_feature->SetGeometry(&voronoi_geom);
    voronoi_layer_->CreateFeature(voronoi_feature);
}