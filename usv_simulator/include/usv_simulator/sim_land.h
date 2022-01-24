#pragma once

#include "geometry_msgs/PolygonStamped.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "geotf/geodetic_converter.h"

#include <gdal/ogrsf_frmts.h>

class SimulatedLand{
    public:
        SimulatedLand(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Publisher poly_pub_;
        std::string path_;
        
        jsk_recognition_msgs::PolygonArray polygon_array_;
        geometry_msgs::PolygonStamped polygon;
        geometry_msgs::Point32 point;
        
        geotf::GeodeticConverter converter;

        OGRSFDriverH gis_driver_;

        void loadPolygons();

};