#pragma once
#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "usv_mission_planner/hybrid_astar.h"
#include "boost/filesystem.hpp"
#include "ctime"
#include "gdal/ogrsf_frmts.h"
#include "tf/transform_datatypes.h"
#include "usv_mission_planner/search.h"
#include "usv_map/map_preprocessor.h"

class MissionPlanner{
    public:
        MissionPlanner(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Publisher path_pub_;
        ros::Publisher speed_pub_;
        ros::Publisher region_available_pub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber goal_sub_;
        ros::ServiceServer search_service_;
        
        Quadtree* tree_;
        ModelLibrary::Viknes830* vessel_model_;
        HybridAStar* search_alg_;
        MapService* map_service_;

        //Mission region
        OGRPoint lower_left_;
        OGRPoint upper_right_;

        std::vector<extendedVertex*> path_;

        //Received data
        nav_msgs::Odometry latest_odom_;

        //Parameters
        std::string mission_name_;
        bool load_from_gpx_;
        std::string gpx_name_;
        bool preprocessed_map_;
        std::string map_name_;
        double desired_speed_;
        std::vector<double> mission_region_extent_;

        //Storage
        std::string mission_path_;

        void setStart(double lon, double lat, double heading);
        void setGoal(double lon, double lat);
        bool search(usv_mission_planner::search::Request &req, usv_mission_planner::search::Response &res);
        void publishPath();
        void publishSpeed();
        void odomCb(const nav_msgs::Odometry& odom);
        void savePath();
};

class MissionPlannerClient{
    public:
        MissionPlannerClient(ros::NodeHandle& nh);
        void searchFromOdom(double goal_lon, double goal_lat, bool publish_path=true);
        void searchFromCustom(double start_lon,double start_lat, double start_heading, double goal_lon, double goal_lat, bool publish_path=true);
        
    private:
        ros::NodeHandle nh_;
        ros::ServiceClient search_client_;
};