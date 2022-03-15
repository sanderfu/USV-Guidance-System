#include "usv_map/skeleton_generator.h"

VoronoiSkeletonGenerator::VoronoiSkeletonGenerator(std::string layername, GDALDataset* ds, Quadtree* tree, MapService* map_service, GeographicLib::Geodesic* geod):
ds_(ds),
tree_(tree),
map_service_(map_service),
geod_(geod){
    in_layer_ = ds_->GetLayerByName(layername.c_str());
    ds_->CreateLayer("voronoi_skeleton",nullptr,wkbMultiLineString);
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

    OGRFeature* voronoi_feature = OGRFeature::CreateFeature(voronoi_layer_->GetLayerDefn());
    voronoi_feature->SetFID(0);
    OGRMultiLineString voronoi_geom;

    OGRPoint check_point_a;
    OGRPoint check_point_b;
    OGRGeometry* geom;
    int total_edges = 0;
    int fast_check_edges = 0;
    int branches_pruned = 0;

    //std::unordered_map<std::pair<double,double>,bool,hash_pair> contained_in_geometry_lookup;

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
        
        if(abs(edge_length/diff)<=1.5){
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
    }
    voronoi_feature->SetGeometry(&voronoi_geom);
    voronoi_layer_->CreateFeature(voronoi_feature);
    jcv_diagram_free( &diagram );
}