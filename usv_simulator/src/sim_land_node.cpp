#include "usv_simulator/sim_land.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"land");
    ros::NodeHandle land_nh("land");

    SimulatedLand sim_land(land_nh);
    ros::spinOnce();
    ros::shutdown();
}