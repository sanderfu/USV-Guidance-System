#include "usv_mission_planner/mission_planner.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"mission_planner_node");
    ros::NodeHandle nh;
    MissionPlanner mission_planner(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
}