#include "usv_simulator/sim_system_visualization.h"
//Description: Currently the land_sim part of the simulator is only responsible for loading the map once and stick around to latch the map.

int main(int argc, char** argv){
    ros::init(argc,argv,"system_visualization");
    ros::NodeHandle land_nh;

    SystemVisualization visualization(land_nh);
    ros::spin();
}