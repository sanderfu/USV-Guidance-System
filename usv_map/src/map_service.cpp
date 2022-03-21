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
        ROS_ERROR_STREAM("Intersection geometry not valid");
        ros::shutdown();
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



/**
 * @brief Construct a new Map Service Server:: Map Service Server object
 * 
 * @details Advertises the service and loads the sqlite check db
 * 
 * @param nh Ros nodehandle.
 */
MapServiceServer::MapServiceServer(const ros::NodeHandle& nh) : nh_(nh){
    std::string mission_region;
    bool parameter_load_error = false;
    if(!ros::param::get("mission_planner/map_name",mission_region)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
    db_path_ = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region+"/"+"region.sqlite";
    GDALAllRegister();
    ds_ = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_ == NULL){
        ROS_ERROR_STREAM("MapServiceServer: Failed to open map db");
        ros::shutdown();
    }

    intersect_service_ = nh_.advertiseService("/map/intersects", &MapServiceServer::intersects, this);
    distance_service_ = nh_.advertiseService("/map/distance", &MapServiceServer::distance, this);

    driver_mem_ = GetGDALDriverManager()->GetDriverByName("Memory");
    memory_ds_ = driver_mem_->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    OGRFeature* feat;
    for(auto&& layer: ds_->GetLayers()){
        if(!(std::string(layer->GetName())=="collision_dissolved" || std::string(layer->GetName())=="caution_dissolved")) continue;
        OGRLayer* multi_layer = memory_ds_->CreateLayer(layer->GetName(),layer->GetSpatialRef(),wkbMultiPolygon);
        OGRFeature* multi_feature = OGRFeature::CreateFeature(multi_layer->GetLayerDefn());
        OGRMultiPolygon multi_poly;
        layer->ResetReading();
            while((feat = layer->GetNextFeature()) != NULL){
                multi_poly.addGeometry(feat->GetGeometryRef());
                OGRFeature::DestroyFeature(feat);
            }
        multi_feature->SetGeometry(&multi_poly);
        multi_feature_map_.insert(std::make_pair(layer->GetName(),multi_feature));
        multi_layer->CreateFeature(multi_feature);
    }

    distance_time = 0.0;
}

/**
 * @brief Service to check intersection between db and geometries
 * 
 * @details Whenever the service is requested, the function checks 
 * for collision with the requested type in the check_db
 * (collision or caution currently)
 * 
 * @param req Service request, see service description.
 * @param res Service response, boolean intersection flag.
 * @return true If function called worked normally
 * @return false If function call failed for any reason.
 */
bool MapServiceServer::intersects(usv_map::intersect::Request& req, usv_map::intersect::Response& res){
    OGRLayer* layer; 
    switch(static_cast<LayerID>(req.intersect_type)){
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

    OGRGeometry* input_geom;
    switch (static_cast<OGRwkbGeometryType>(req.geom_type))
    {
    case OGRwkbGeometryType::wkbLineString:
        input_geom = new OGRLineString;
        break;
    case OGRwkbGeometryType::wkbPoint:
        input_geom = new OGRPoint;
        break;
    default:
        ROS_ERROR_STREAM("Intersection handling for geometry type " << req.geom_type << " not implemented");
        return false;
    }
    input_geom->assignSpatialReference(layer->GetSpatialRef());
    const char* wkt = req.path_wkt.c_str();
    input_geom->importFromWkt(&wkt);

    if(!input_geom->IsValid()){
        ROS_ERROR_STREAM("Intersection geometry not valid");
        ros::shutdown();
        return false;
    }
    layer->ResetReading();
    res.intersects=false;
    while(( feat_ = layer->GetNextFeature())!=NULL){
        OGRGeometry* geom = feat_->GetGeometryRef();
        if(input_geom->Intersects(geom)){
            res.intersects=true;
            break;
        }
        OGRFeature::DestroyFeature(feat_);
    }
    
    OGRFeature::DestroyFeature(feat_);
    OGRGeometryFactory::destroyGeometry(input_geom);
    return true;
}

bool MapServiceServer::distance(usv_map::distance::Request& req, usv_map::distance::Response& res){
    OGRLayer* layer;
    switch(static_cast<LayerID>(req.layer_id)){
    case LayerID::COLLISION:
        layer = memory_ds_->GetLayerByName("collision_dissolved");
        break;
    case LayerID::CAUTION:
        layer = memory_ds_->GetLayerByName("caution_dissolved");
        break;
    default:
        ROS_ERROR_STREAM("Invalid LayerType");
        return false;
    }
    OGRPoint point;    
    point.setX(req.lon);
    point.setY(req.lat);
    ros::Time start = ros::Time::now();
    res.distance = std::min(layer->GetFeature(0)->GetGeometryRef()->Distance(&point),0.005);
    ros::Time end = ros::Time::now();
    distance_time+=ros::Duration(end-start).toSec();
    std::cout << "Total time so far: " << distance_time << std::endl;
    return true;
}

/**
 * @brief Construct a new Map Service Client:: Map Service Client object
 * 
 * @param nh 
 */
MapServiceClient::MapServiceClient(ros::NodeHandle* nh) : nh_(nh){
    intersects_service = nh_->serviceClient<usv_map::intersect>("/map/intersects");
    distance_service = nh_->serviceClient<usv_map::distance>("/map/distance");
}

/**
 * @brief Call the intersects service
 * 
 * @param geom Geometry to check agains the spatial db
 * @param intersect_type Define if collision or caution
 * @return true If is intersection
 * @return false If is not intersection
 */
bool MapServiceClient::intersects(OGRGeometry* geom, LayerID intersect_type){
    usv_map::intersect srv;
    srv.request.geom_type = static_cast<uint8_t>(geom->getGeometryType());
    srv.request.intersect_type = intersect_type;
    char* wkt;
    geom->exportToWkt(&wkt);
    srv.request.path_wkt = static_cast<std::string>(wkt);
    if(!intersects_service.call(srv)){
        ROS_ERROR_STREAM("Failed to check intersection");
    }
    return srv.response.intersects;
}

double MapServiceClient::distance(double lon, double lat, LayerID layer_id){
    usv_map::distance srv;
    srv.request.lon = lon;
    srv.request.lat = lat;
    srv.request.layer_id = layer_id;
    if(!distance_service.call(srv)){
        ROS_ERROR_STREAM("Failed to check distance");
    }
    return srv.response.distance;
}

/**
 * @brief Check if geometry collides with land
 * 
 * @param geom The geometry to check
 * @return true If collision
 * @return false If not collision
 */
bool MapServiceClient::collision(OGRGeometry* geom){
    return intersects(geom,LayerID::COLLISION);
}

/**
 * @brief Check if geometry intersects with caution area
 * 
 * @param geom Geometry to check
 * @return true If intersects caution area
 * @return false If does not intersect caution area
 */
bool MapServiceClient::caution(OGRGeometry* geom){
    return intersects(geom,LayerID::CAUTION);
}
