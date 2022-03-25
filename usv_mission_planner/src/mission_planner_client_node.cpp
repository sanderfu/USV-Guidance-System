#include "usv_mission_planner/mission_planner.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"mission_planner_client_node");
    ros::NodeHandle nh;
    MissionPlannerClient mission_planner_client(nh);
    std::cout << "Wait for region" << std::endl;
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh);
    std::cout << "Waiting 2 seconds before requesting path search" << std::endl;
    ros::Duration(2).sleep();
    std::cout << "Start search" << std::endl;
    mission_planner_client.searchFromOdom(9.53574,63.62217);
    //mission_planner_client.searchFromOdom(9.30736,63.69028);
    std::cout << "Search done" << std::endl;

    ros::spin();
}