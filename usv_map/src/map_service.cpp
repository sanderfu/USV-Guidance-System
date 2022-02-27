#include "usv_map/map_service.h"

MapService::MapService(){
    std::string db_path_ = ros::package::getPath("usv_simulator")+"/maps/check_db.sqlite";
    GDALAllRegister();
    ds_ = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_ == NULL)
    {
        ROS_ERROR_STREAM("MapService: Failed to open map db");
        ros::shutdown();
    }

    std::string db_lite_path=ros::package::getPath("usv_simulator") + "/maps/check_db_lite.sqlite";
    GDALDataset* ds_lite_ = (GDALDataset*) GDALOpenEx(db_lite_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_lite_ == NULL)
    {
        ROS_ERROR_STREAM("MapService: Failed to open map db");
        ros::shutdown();
    }

    driver_mem_ = GetGDALDriverManager()->GetDriverByName("Memory");
    ds_in_mem_ = driver_mem_->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    OGRFeature* feat;
    for(auto&& layer: ds_lite_->GetLayers()){
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

double MapService::distance(double lon,double lat,LayerID layer_id){
    OGRLayer* layer;
    switch(layer_id){
    case LayerID::COLLISION:
        layer = ds_in_mem_->GetLayerByName("collision_dissolved");
        break;
    case LayerID::CAUTION:
        layer = ds_in_mem_->GetLayerByName("caution_dissolved");
        break;
    default:
        ROS_ERROR_STREAM("Invalid LayerType");
        return false;
    } 
    distance_point_.setX(lon);
    distance_point_.setY(lat);
    OGRFeature* feat = layer->GetFeature(0);
    double distance = std::min(feat->GetGeometryRef()->Distance(&distance_point_),0.005);
    OGRFeature::DestroyFeature(feat);
    return distance;
}




/**
 * @brief Construct a new Map Service Server:: Map Service Server object
 * 
 * @details Advertises the service and loads the sqlite check db
 * 
 * @param nh Ros nodehandle.
 */
MapServiceServer::MapServiceServer(const ros::NodeHandle& nh) : nh_(nh){
    db_path_ = ros::package::getPath("usv_simulator");
    db_path_.append("/maps/check_db.sqlite");
    GDALAllRegister();
    ds_ = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_ == NULL){
        ROS_ERROR_STREAM("MapServiceServer: Failed to open map db");
        ros::shutdown();
    }


    intersect_service_ = nh_.advertiseService("/map/intersects", &MapServiceServer::intersects, this);
    distance_service_ = nh_.advertiseService("/map/distance", &MapServiceServer::distance, this);

    std::string db_lite_path=ros::package::getPath("usv_simulator") + "/maps/check_db_lite.sqlite";
    GDALDataset* ds_lite = (GDALDataset*) GDALOpenEx(db_lite_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if( ds_lite == NULL)
    {
        ROS_ERROR_STREAM("MapServiceServer: Failed to open map db");
        ros::shutdown();
    }

    driver_mem_ = GetGDALDriverManager()->GetDriverByName("Memory");
    memory_ds_ = driver_mem_->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    OGRFeature* feat;
    for(auto&& layer: ds_lite->GetLayers()){
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
