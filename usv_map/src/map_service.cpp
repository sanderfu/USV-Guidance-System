#include "usv_map/map_service.h"

MapService::MapService(std::string mission_region){
    GDALAllRegister();
    std::string mission_path = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region+"/";

    if(!boost::filesystem::exists(mission_path)){
        ROS_WARN_STREAM("Tried to use preprocessed mission region: "<< mission_region << " which has not been defined");
        ros::shutdown();
    }

    std::string db_path_ = mission_path+"region.sqlite";
    std::string db_detailed_path = mission_path+"region_detailed.sqlite";
    GDALAllRegister();
    ds_ = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    ds_detailed_ = (GDALDataset*) GDALOpenEx(db_detailed_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_ == NULL || ds_detailed_ == NULL)
    {
        ROS_ERROR_STREAM("MapService: Failed to open dataset");
        ros::shutdown();
    }

    driver_mem_ = GetGDALDriverManager()->GetDriverByName("Memory");
    ds_in_mem_ = driver_mem_->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    OGRFeature* feat;
    for(auto&& layer: ds_->GetLayers()){
        if(!(std::string(layer->GetName())=="collision_dissolved" || std::string(layer->GetName())=="caution_dissolved")) continue;
        OGRLayer* multi_layer = ds_in_mem_->CreateLayer(layer->GetName(),layer->GetSpatialRef(),wkbMultiPolygon);
        OGRFeature* multi_feature = OGRFeature::CreateFeature(multi_layer->GetLayerDefn());
        OGRMultiPolygon multi_poly;
        layer->ResetReading();
            while((feat = layer->GetNextFeature()) != NULL){
                multi_poly.addGeometry(feat->GetGeometryRef());
                OGRFeature::DestroyFeature(feat);
            }
        multi_feature->SetGeometry(&multi_poly);
        multi_layer->CreateFeature(multi_feature);
    }

    OGRLineString* mission_region_boundary = ds_->GetLayerByName("mission_region")->GetFeature(1)->GetGeometryRef()->getBoundary()->toLineString();
    mission_region_boundary->getPoint(0,&lower_left_);
    mission_region_boundary->getPoint(2,&upper_right_);

    ds_in_mem_->CopyLayer(ds_->GetLayerByName("voronoi"),"voronoi");

    //Load parameters
    bool parameter_load_error = false;
    if(!ros::param::get("map_service/voronoi_field/alpha",alpha_)) parameter_load_error = true;
    if(!ros::param::get("map_service/distance/default_saturation",default_saturation_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
    
}

MapService::MapService(GDALDataset* ds, GDALDataset* ds_detailed):
ds_(ds),
ds_detailed_(ds_detailed){
    driver_mem_ = GetGDALDriverManager()->GetDriverByName("Memory");
    //driver_mem_ = GetGDALDriverManager()->GetDriverByName("SQLite");
    ds_in_mem_ = driver_mem_->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    //ds_in_mem_ = driver_mem_->Create((ros::package::getPath("usv_map")+"/data/debug/debug.sqlite").c_str(),0,0,0,GDT_Unknown,NULL);
    OGRFeature* feat;
    for(auto&& layer: ds_->GetLayers()){
        if(!(std::string(layer->GetName())=="collision_dissolved" || std::string(layer->GetName())=="caution_dissolved")) continue;
        OGRLayer* multi_layer = ds_in_mem_->CreateLayer(layer->GetName(),layer->GetSpatialRef(),wkbMultiPolygon);
        OGRFeature* multi_feature = OGRFeature::CreateFeature(multi_layer->GetLayerDefn());
        OGRMultiPolygon multi_poly;
        layer->ResetReading();
            while((feat = layer->GetNextFeature()) != NULL){
                multi_poly.addGeometry(feat->GetGeometryRef());
                OGRFeature::DestroyFeature(feat);
            }
        multi_feature->SetGeometry(&multi_poly);
        multi_layer->CreateFeature(multi_feature);
    }

    OGREnvelope layer_envelope;
    ds_in_mem_->ResetReading();
    lower_left_.setX(INFINITY);
    lower_left_.setY(INFINITY);

    upper_right_.setX(-INFINITY);
    upper_right_.setY(-INFINITY);
    for(auto&& layer: ds_in_mem_->GetLayers()){
        layer->GetExtent(&layer_envelope);
        if(layer_envelope.MinX<lower_left_.getX() && layer_envelope.MinY<lower_left_.getY()){
            lower_left_.setX(layer_envelope.MinX);
            lower_left_.setY(layer_envelope.MinY);
        }
        if(layer_envelope.MaxX>upper_right_.getX() && layer_envelope.MaxY>upper_right_.getY()){
            upper_right_.setX(layer_envelope.MaxX);
            upper_right_.setY(layer_envelope.MaxY);
        }
    }

    //Load parameters
    bool parameter_load_error = false;
    if(!ros::param::get("map_service/voronoi_field/alpha",alpha_)) parameter_load_error = true;
    if(!ros::param::get("map_service/distance/default_saturation",default_saturation_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
}

bool MapService::intersects(OGRGeometry* input_geom, LayerID layer_id){
    OGRLayer* layer; 
    switch(layer_id){
    case LayerID::COLLISION:
        layer = ds_->GetLayerByName("collision_dissolved");
        break;
    case LayerID::CAUTION:
        layer = ds_->GetLayerByName("caution_dissolved");
        break;
    default:
        ROS_ERROR_STREAM("Invalid LayerType");
        return false;
    }

    if(!input_geom->IsValid()){
        //This is commonly when most all path elements are the same value due to no velocity and no desired velocity in sim.
        return false;
    }

    OGRFeature* feat;
    double total_area = 0;

    std::vector<OGRGeometry*> geometries_to_check;
    std::vector<OGRFeature*> related_features;

    OGREnvelope input_env;
    input_geom->getEnvelope(&input_env);
    OGREnvelope check_env;

    layer->ResetReading();
    while((feat = layer->GetNextFeature()) != NULL){
        feat->GetGeometryRef()->getEnvelope(&check_env);
        if(check_env.Intersects(input_env)){
            geometries_to_check.push_back(feat->GetGeometryRef()); 
            related_features.push_back(feat);
        } else{
            OGRFeature::DestroyFeature(feat);
        }
    }
    bool intersection=false;
    #pragma omp parallel for reduction(+:total_area)
        for(auto geom_it = geometries_to_check.begin(); geom_it!=geometries_to_check.end();geom_it++){
            if(intersection){
                OGRFeature::DestroyFeature(related_features[geom_it-geometries_to_check.begin()]);
                continue;
            }
            if(input_geom->Intersects(*geom_it)){
                intersection=true;
            }
            OGRFeature::DestroyFeature(related_features[geom_it-geometries_to_check.begin()]);  
        }
    
    return intersection;
}

double MapService::distance(double lon,double lat,LayerID layer_id,double max_distance){
    if(max_distance==-1){
        max_distance = default_saturation_;
    }
    OGRLayer* layer;
    switch(layer_id){
    case LayerID::COLLISION:
        layer = ds_in_mem_->GetLayerByName("collision_dissolved");
        break;
    case LayerID::CAUTION:
        layer = ds_in_mem_->GetLayerByName("caution_dissolved");
        break;
    case LayerID::VORONOI:
        layer = ds_in_mem_->GetLayerByName("voronoi");
        break;
    default:
        ROS_ERROR_STREAM("Invalid LayerType");
        return false;
    } 
    if(layer==NULL){
        ROS_ERROR_STREAM("Unable to load layer");
        return -1;
    }
    distance_point_.setX(lon);
    distance_point_.setY(lat);
    OGRFeature* feat = layer->GetFeature(0);
    if(feat==NULL){
        ROS_ERROR_STREAM("Unable to get feature");
        return -1;
    }
    double distance = std::min(feat->GetGeometryRef()->Distance(&distance_point_),max_distance);
    //double distance = feat->GetGeometryRef()->Distance(&distance_point_);
    OGRFeature::DestroyFeature(feat);
    return distance;
}

double MapService::voronoi_field(double lon, double lat){
    double distance_to_land = distance(lon,lat,LayerID::COLLISION);
    double distance_voronoi = distance(lon,lat,LayerID::VORONOI);
    return (alpha_/(alpha_+distance_to_land))*(distance_voronoi/(distance_voronoi+distance_to_land))*(pow(distance_to_land-default_saturation_,2)/pow(default_saturation_,2));
}

/**
 * @brief Get all features present at a geudetic location.
 * 
 * @warning The caller is responsible to destory the returned geometry pointers using OGRFeature::DestroyFeature!
 * 
 * @param lon Longitude
 * @param lat Latitude
 * @return std::vector<OGRFeature*> List of features
 */
std::vector<OGRFeature*> MapService::getFeatures(double lon, double lat){
    OGRPoint interest_point(lon,lat);
    std::vector<OGRFeature*> features_intersecting;

    std::vector<OGRGeometry*> geometries_to_check;
    std::vector<OGRFeature*> related_features;

    OGREnvelope interest_env;
    interest_point.getEnvelope(&interest_env);
    OGREnvelope check_env;

    OGRLayer* layer;
    OGRFeature* feat;

    for (OGRLayer* layer: (ds_detailed_->GetLayers())){
        while((feat = layer->GetNextFeature()) != NULL){
            feat->GetGeometryRef()->getEnvelope(&check_env);
            if(check_env.Intersects(interest_env)){
                geometries_to_check.push_back(feat->GetGeometryRef()); 
                related_features.push_back(feat);
            } else{
                OGRFeature::DestroyFeature(feat);
            }
        }
    }
    #pragma omp parallel for
        for(auto geom_it = geometries_to_check.begin(); geom_it!=geometries_to_check.end();geom_it++){
            if(interest_point.Intersects(*geom_it)){
                features_intersecting.push_back(related_features[geom_it-geometries_to_check.begin()]);
            } 
        }
    
    return features_intersecting;

}

std::pair<OGRPoint, OGRPoint> MapService::getMapExtent(){
    return std::make_pair(lower_left_,upper_right_);
}

GDALDataset* MapService::getDataset(){
    return ds_;
}