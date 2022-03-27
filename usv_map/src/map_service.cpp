#include "usv_map/map_service.h"

MapService::MapService(std::string mission_region){
    GDALAllRegister();
    std::string mission_path = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region+"/";

    if(!boost::filesystem::exists(mission_path)){
        ROS_WARN_STREAM("Tried to use preprocessed mission region: "<< mission_region << " which has not been defined");
        ros::shutdown();
    }

    std::string db_path_ = mission_path+"region.sqlite";
    std::cout << db_path_ << std::endl;
    GDALAllRegister();
    ds_ = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_ == NULL)
    {
        ROS_ERROR_STREAM("MapService: Failed to open map db");
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

MapService::MapService(GDALDataset* ds):
ds_(ds){
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
    layer->ResetReading();
    OGRFeature* feat;
    while(( feat = layer->GetNextFeature())!=NULL){
        OGRGeometry* geom = feat->GetGeometryRef();
        if(input_geom->Intersects(geom)){
            OGRFeature::DestroyFeature(feat);
            return true;
        }
        OGRFeature::DestroyFeature(feat);
    }
    
    OGRFeature::DestroyFeature(feat);
    return false;
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

std::pair<OGRPoint, OGRPoint> MapService::getMapExtent(){
    return std::make_pair(lower_left_,upper_right_);
}

GDALDataset* MapService::getDataset(){
    return ds_;
}