#pragma once

#include "usv_simulator/PolygonArray64.h"
#include "usv_simulator/Polygon64.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "geotf/geodetic_converter.h"
#include <gdal/ogrsf_frmts.h>
#include "ros/topic.h"
#include "std_msgs/Bool.h"

class SimulatedLand{
    public:
        SimulatedLand(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Publisher poly_pub_;
        std::string path_;
        
        usv_simulator::PolygonArray64 polygon_array_;
        usv_simulator::Polygon64 polygon_;
        geometry_msgs::Point point_;
        
        geotf::GeodeticConverter converter_;

        OGRSFDriverH gis_driver_;

        void loadPolygons();

};