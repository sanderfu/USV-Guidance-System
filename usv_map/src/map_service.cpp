#include "usv_map/map_service.h"

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

bool MapServiceServer::intersects(usv_map::intersect::Request& req, usv_map::intersect::Response& res){
    //std::cout << "Intersects service callback" << std::endl;
    OGRLayer* collision_layer = ds_->GetLayerByName("collision_dissolved");
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
    }
    input_geom->assignSpatialReference(collision_layer->GetSpatialRef());
    const char* wkt = req.path_wkt.c_str();
    input_geom->importFromWkt(&wkt);

    if(!input_geom->IsValid()){
        ROS_ERROR_STREAM("Intersection geometry not valid");
        ros::shutdown();
        return false;
    }
    /*
    for(int i=0; i<path.getNumPoints();i++){
        std::cout << path.getX(i) << " " << path.getY(i) << std::endl;
    }
    */
    collision_layer->ResetReading();
    OGRFeature* feat;
    while(( feat = collision_layer->GetNextFeature())!=NULL){
        OGRGeometry *geom = feat->GetGeometryRef();
        if(input_geom->Intersects(geom)){
            res.intersects=true;
            return true;
        }
    }
    res.intersects=false;
    return true;
}

MapServiceClient::MapServiceClient(ros::NodeHandle* nh) : nh_(nh){
    intersects_service = nh_->serviceClient<usv_map::intersect>("/map/intersects");
}

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

bool MapServiceClient::collision(OGRGeometry* geom){
    return intersects(geom,IntersectType::COLLISION);
}

bool MapServiceClient::caution(OGRGeometry* geom){
    return intersects(geom,IntersectType::CAUTION);
}
