#pragma once

#include "jsk_recognition_msgs/PolygonArray.h"
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
        geometry_msgs::PolygonStamped polygon_;
        geometry_msgs::Point32 point_;
        
        geotf::GeodeticConverter converter_;

        OGRSFDriverH gis_driver_;

        void loadPolygons();

};