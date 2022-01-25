#include "usv_simulator/sim_land.h"

#include <iostream>
#include <fstream>

SimulatedLand::SimulatedLand(const ros::NodeHandle& nh){
    path_ = ros::package::getPath("usv_simulator");
    path_.append("/maps/check_db.sqlite");

    polygon.header.frame_id="map";
    poly_pub_ = nh_.advertise<jsk_recognition_msgs::PolygonArray>("/sim/land",1,true);

    converter.addFrameByENUOrigin("local",40.5612,-73.9761,0);
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
    std::ofstream myfile;
    myfile.open("polygon.txt");
    int counter = 0;
    while(feat!=NULL){
        polygon.polygon.points.clear();
        //https://stackoverflow.com/questions/18747370/how-to-extract-vertexes-of-geometries-in-esri-shapefiles-using-ogr-library-with
        OGRLinearRing* ext_ring = feat->GetGeometryRef()->toPolygon()->getExteriorRing();

        for(int i=0;i<ext_ring->getNumPoints()-1;i++){
            //std::cout << ext_ring->getNumPoints() <<std::endl;
            Eigen::Vector3d global_coord(ext_ring->getX(i),ext_ring->getY(i),0);
            Eigen::Vector3d local_coord;
            converter.convert("global",global_coord,"local",&local_coord);
            point.x = local_coord(0);
            point.y = local_coord(1);
            polygon.polygon.points.push_back(point);
        }
        if(ext_ring->getNumPoints()>1){
            polygon_array_.polygons.push_back(polygon);
            polygon_array_.labels.push_back(label++);
            polygon_array_.likelihood.push_back(1);
            counter++;
        }

        feat = collision_layer->GetNextFeature();
    }

    polygon_array_.header.frame_id="map";
    std::cout << "Polygon array size: " << polygon_array_.polygons.size() << std::endl;
    poly_pub_.publish(polygon_array_);
    
    ROS_INFO_STREAM("Loaded polygons from db");
}