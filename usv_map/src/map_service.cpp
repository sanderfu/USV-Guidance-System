#include "usv_map/map_service.h"
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
    if( ds_ == NULL)
    {
        ROS_ERROR_STREAM("MapServiceServer: Failed to open map db");
        ros::shutdown();
    }
    intersect_service_ = nh_.advertiseService("/map/intersects", &MapServiceServer::intersects, this);
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
    switch(static_cast<IntersectType>(req.intersect_type)){
    case IntersectType::COLLISION:
        layer = ds_->GetLayerByName("collision_dissolved");
        break;
    case IntersectType::CAUTION:
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

/**
 * @brief Construct a new Map Service Client:: Map Service Client object
 * 
 * @param nh 
 */
MapServiceClient::MapServiceClient(ros::NodeHandle* nh) : nh_(nh){
    intersects_service = nh_->serviceClient<usv_map::intersect>("/map/intersects");
}

/**
 * @brief Call the intersects service
 * 
 * @param geom Geometry to check agains the spatial db
 * @param intersect_type Define if collision or caution
 * @return true If is intersection
 * @return false If is not intersection
 */
bool MapServiceClient::intersects(OGRGeometry* geom, IntersectType intersect_type){
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

/**
 * @brief Check if geometry collides with land
 * 
 * @param geom The geometry to check
 * @return true If collision
 * @return false If not collision
 */
bool MapServiceClient::collision(OGRGeometry* geom){
    return intersects(geom,IntersectType::COLLISION);
}

/**
 * @brief Check if geometry intersects with caution area
 * 
 * @param geom Geometry to check
 * @return true If intersects caution area
 * @return false If does not intersect caution area
 */
bool MapServiceClient::caution(OGRGeometry* geom){
    return intersects(geom,IntersectType::CAUTION);
}
