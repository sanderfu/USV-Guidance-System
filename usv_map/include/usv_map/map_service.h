#pragma once

#include "ros/ros.h"
#include <ros/package.h>
#include "gdal/ogrsf_frmts.h"
#include <GeographicLib/Geodesic.hpp>
#include "usv_map/intersect.h"

class MapServiceServer {
    public:
        MapServiceServer(const ros::NodeHandle& nh);
        bool intersects(usv_map::intersect::Request& req, usv_map::intersect::Response& res);
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer intersect_service_;
        
        std::string db_path_;
        GDALDataset* ds_;


};
