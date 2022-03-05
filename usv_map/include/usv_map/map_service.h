#pragma once

#include "ros/ros.h"
#include <ros/package.h>
#include "gdal/ogrsf_frmts.h"
#include <GeographicLib/Geodesic.hpp>
#include "usv_map/intersect.h"
#include "usv_map/distance.h"
#include "unordered_map"
#include "boost/filesystem.hpp"

enum LayerID {COLLISION, CAUTION, VORONOI};

class MapService {
    public:
        MapService(std::string mission_region);
        bool intersects(OGRGeometry* geom, LayerID layer_id);
        double distance(double lon,double lat,LayerID layer_id,double max_distance=-1);
        double voronoi_field(double lon, double lat);
        std::pair<OGRPoint, OGRPoint> getMapExtent();
        GDALDataset* getDataset();
    private:
        GDALDataset* ds_;
        GDALDataset* ds_in_mem_;
        GDALDriver* driver_mem_;
        OGRPoint distance_point_;

        //Mission region extent
        OGRPoint lower_left_;
        OGRPoint upper_right_;


        //Parameters
        double alpha_;
        double default_saturation_;
};

class MapServiceServer {
    public:
        MapServiceServer(const ros::NodeHandle& nh);
        bool intersects(usv_map::intersect::Request& req, usv_map::intersect::Response& res);
        bool distance(usv_map::distance::Request& req, usv_map::distance::Response& res);
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer intersect_service_;
        ros::ServiceServer distance_service_;
        
        std::string db_path_;
        GDALDataset* ds_;

        OGRFeature* feat_;

        GDALDriver* driver_mem_;
        GDALDataset* memory_ds_;
        std::unordered_map<std::string,OGRFeature*> multi_feature_map_;
        double distance_time;

};

class MapServiceClient {
    public: 
        MapServiceClient(ros::NodeHandle* nh);
        bool collision(OGRGeometry* geom);
        bool caution(OGRGeometry* geom);
        double distance(double lon,double lat,LayerID layer_id);
    private:
        ros::NodeHandle* nh_;
        ros::ServiceClient intersects_service;
        ros::ServiceClient distance_service;
        bool intersects(OGRGeometry* geom, LayerID intersect_type);
};