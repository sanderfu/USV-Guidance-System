#include "usv_simulator/sim_system_visualization.h"

SystemVisualization::SystemVisualization(const ros::NodeHandle& nh): nh_(nh){
    geo_waypoint_viz_ = nh_.advertise<visualization_msgs::Marker>("systemViz/waypoints",1,false);
    los_vector_viz_ = nh_.advertise<visualization_msgs::Marker>("systemViz/los",1,false);

    std::vector<double> global_position_vec;
    if(!nh_.getParam("sim_origin",global_position_vec)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
    }

    converter_.addFrameByENUOrigin("local",global_position_vec[1],global_position_vec[0],0);
    converter_.addFrameByEPSG("global",4326);
    if(!converter_.canConvert("global","local")){
        ROS_ERROR_STREAM_NAMED("sim_land","Frame conversion error");
        ros::shutdown();
    }

    markerInit();

    odom_sub_ = nh_.subscribe("odom",1,&SystemVisualization::odomCb,this);
    geo_waypoint_sub_ = nh_.subscribe("mission_planner/geo_waypoint",1,&SystemVisualization::waypointCb,this);
    los_vector_sub_ = nh_.subscribe("los/setpoint",1,&SystemVisualization::losVectorCb,this);
    //reinit_sub_ = nh_.subscribe("mc/system_reinit",1,&SystemVisualization::reinitCb,this);
}

void SystemVisualization::markerInit(){
    wpts_marker_.header.frame_id = "map";
    wpts_marker_.header.stamp = ros::Time::now();
    wpts_marker_.ns = "waypoints";
    wpts_marker_.id = 0;
    wpts_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    wpts_marker_.scale.x = 10;
    wpts_marker_.scale.y = 10;
    wpts_marker_.scale.z = 10;
    wpts_marker_.action = visualization_msgs::Marker::ADD;
    wpts_marker_.color.a = 1.0;
    wpts_marker_.color.r = 1.0;
    wpts_marker_.pose.orientation.w = 1.0; //To remove unitiliized quaternion error message RVIZ

    los_vector_marker_.header.frame_id = "map";
    los_vector_marker_.header.stamp = ros::Time::now();
    los_vector_marker_.ns = "los_vector";
    los_vector_marker_.type = visualization_msgs::Marker::ARROW;
    los_vector_marker_.scale.x = 50;
    los_vector_marker_.scale.y = 1;
    los_vector_marker_.scale.z = 1;
    los_vector_marker_.color.a = 1.0;
    los_vector_marker_.color.b = 1.0;

}

void SystemVisualization::odomCb(const nav_msgs::Odometry& msg){
    latest_odom_ = msg;
}

void SystemVisualization::waypointCb(const geometry_msgs::Pose& msg){
    Eigen::Vector3d wpt_wgs(msg.position.x,msg.position.y,msg.position.z);
    Eigen::Vector3d wpt_enu;
    converter_.convert("global",wpt_wgs,"local",&wpt_enu);
    geometry_msgs::Point point;
    point.x = wpt_enu.x();
    point.y = wpt_enu.y();
    wpts_marker_.points.push_back(point);
    geo_waypoint_viz_.publish(wpts_marker_);
}

void SystemVisualization::losVectorCb(const geometry_msgs::Twist& msg){
    Eigen::Vector3d position_wgs(latest_odom_.pose.pose.position.x,latest_odom_.pose.pose.position.y,latest_odom_.pose.pose.position.z);
    Eigen::Vector3d position_enu;
    converter_.convert("global",position_wgs,"local",&position_enu);
    los_vector_marker_.pose.position.x = position_enu[0];
    los_vector_marker_.pose.position.y = position_enu[1];

    tf::Quaternion q;
    q.setRPY(0,0,msg.angular.z);
    tf::quaternionTFToMsg(q,los_vector_marker_.pose.orientation);
    los_vector_viz_.publish(los_vector_marker_);
}

void SystemVisualization::reinitCb(const usv_msgs::reinit& msg){
    wpts_marker_.points.clear();
    geo_waypoint_viz_.publish(wpts_marker_);
}

