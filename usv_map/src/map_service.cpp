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
    OGRLineString path;
    path.assignSpatialReference(collision_layer->GetSpatialRef());
    const char* wkt = req.path_wkt.c_str();
    path.importFromWkt(&wkt);
    if(!path.IsValid()){
        ROS_ERROR_STREAM("Intersection path not valid");
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
        if(path.Intersects(geom)){
            res.intersects=true;
            return true;
        }
    }
    res.intersects=false;
    return true;
}