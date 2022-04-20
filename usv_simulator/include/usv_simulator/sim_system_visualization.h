#pragma once
#include "ros/ros.h"
#include "usv_msgs/reinit.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "usv_map/geotf_sync.h"
#include "visualization_msgs/Marker.h"

class SystemVisualization{
    public:
        SystemVisualization(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        ros::Subscriber geo_waypoint_sub_;
        ros::Subscriber los_vector_sub_;
        ros::Subscriber reinit_sub_;
        ros::Publisher geo_waypoint_viz_;
        ros::Publisher los_vector_viz_;

        geotf::GeodeticConverter converter_;
        nav_msgs::Odometry latest_odom_;

        //Waypoint marker
        visualization_msgs::Marker wpts_marker_;
        visualization_msgs::Marker los_vector_marker_;

        void odomCb(const nav_msgs::Odometry& msg);
        void waypointCb(const geometry_msgs::Pose& msg);
        void losVectorCb(const geometry_msgs::Twist& msg);
        void reinitCb(const usv_msgs::reinit& msg);

        void markerInit();

};