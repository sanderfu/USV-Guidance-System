#include "usv_mission_planner/mission_planner.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"mission_planner_client_node");
    ros::NodeHandle nh;
    MissionPlannerClient mission_planner_client(nh);

    std::cout << "Start search" << std::endl;
    mission_planner_client.searchFromCustom(-73.999927,40.590175,-M_PI,-73.8443265,40.6415880);
    std::cout << "Search done" << std::endl;

    ros::spin();
}