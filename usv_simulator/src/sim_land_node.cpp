#include "usv_simulator/sim_land.h"
//Description: Currently the land_sim part of the simulator is only responsible for loading the map once and stick around to latch the map.

int main(int argc, char** argv){
    ros::init(argc,argv,"land");
    ros::NodeHandle land_nh("land");

    SimulatedLand sim_land(land_nh);
    ros::spin();
}