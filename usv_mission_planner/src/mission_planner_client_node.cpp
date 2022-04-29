#include "usv_mission_planner/mission_planner.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"mission_planner_client_node");
    ros::NodeHandle nh;

    bool parameter_load_error = false;
    std::string mission_name;
    if(!ros::param::get("mission_name",mission_name)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
    
    MissionPlannerClient mission_planner_client(nh);
    std::cout << "Wait for region" << std::endl;
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh);
    std::cout << "Waiting 2 seconds before requesting path search" << std::endl;
    ros::Duration(2).sleep();
    std::cout << "Start search" << std::endl;
    mission_planner_client.searchFromOdom(-73.839272,40.641694,mission_name);
    //mission_planner_client.searchFromOdom(9.29729,63.54473);
    std::cout << "Search done" << std::endl;

    ros::spin();
}