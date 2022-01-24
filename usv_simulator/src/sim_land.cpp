#include "usv_simulator/sim_land.h"

SimulatedLand::SimulatedLand(const ros::NodeHandle& nh){
    path_ = ros::package::getPath("usv_simulator");
    path_.append("/maps/check_db.sqlite");

    polygon.header.frame_id="map";
    poly_pub_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>("/sim/land",1,true);

    gis_driver_ = OGRGetDriverByName("SQLite");
    loadPolygons();
    converter.addFrameByENUOrigin("local",40.6456,-73.8334,0);
    converter.addFrameByEPSG("global",4326);
    std::cout << "Can convert from global to local: " << converter.canConvert("global","local") << std::endl;
    loadPolygons();
}

void SimulatedLand::loadPolygons(){
    ROS_INFO_STREAM("Starting to load polygons from db");
    GDALAllRegister();
    GDALDataset *ds_ptr_;
    ds_ptr_ = (GDALDataset*) GDALOpenEx(path_.c_str(),GDAL_OF_VECTOR,NULL,NULL,NULL);
    if (ds_ptr_==NULL){
        ROS_ERROR("Failed to open map");
        ros::shutdown();
    }
    OGRLayer* collision_layer = ds_ptr_->GetLayerByName("collision_dissolved");
    OGRFeature* feat = collision_layer->GetNextFeature();
    int label = 0;
    while(feat!=NULL){
        polygon.polygon.points.clear();
        //https://stackoverflow.com/questions/18747370/how-to-extract-vertexes-of-geometries-in-esri-shapefiles-using-ogr-library-with
        OGRLinearRing* ext_ring = feat->GetGeometryRef()->toPolygon()->getExteriorRing();
        
        for(int i=0;i<ext_ring->getNumPoints();i++){
            Eigen::Vector3d global_coord(ext_ring->getX(i),ext_ring->getY(i),0);
            Eigen::Vector3d local_coord;
            converter.convert("global",global_coord,"local",&local_coord);
            point.x = local_coord(0);
            point.y = local_coord(1);
            std::cout << "x: " << point.x << " y: " << point.y << std::endl;
            polygon.polygon.points.push_back(point);
            continue;
        }
        polygon_array_.polygons.push_back(polygon);
        polygon_array_.labels.push_back(label++);
        polygon_array_.likelihood.push_back(1);
        feat = collision_layer->GetNextFeature();
    }

    polygon_array_.header.frame_id="map";
    poly_pub_.publish(polygon_array_);
    
    /*
    for (auto poly_it = polygon_vector_.begin(); poly_it!=polygon_vector_.end(); poly_it++){
        geometry_msgs::PolygonStamped test_poly;
        test_poly.header.frame_id="map";
        geometry_msgs::Point32 point1;
        point1.x = -10;
        point1.y = -10;
        test_poly.polygon.points.push_back(point1);
        geometry_msgs::Point32 point2;
        point2.x = -10;
        point2.y = 10;
        test_poly.polygon.points.push_back(point2);
        geometry_msgs::Point32 point3;
        point3.x = 10;
        point3.y = 10;
        test_poly.polygon.points.push_back(point3);
        geometry_msgs::Point32 point4;
        point4.x = 10;
        point4.y = -10;
        test_poly.polygon.points.push_back(point4);
        geometry_msgs::Point32 point5;
        point5.x = -10;
        point5.y = -10;
        test_poly.polygon.points.push_back(point5);
        poly_pub_.publish(*poly_it);
        ros::Duration(1).sleep();
    }
    */
    
    ROS_INFO_STREAM("Loaded polygons from db");
}