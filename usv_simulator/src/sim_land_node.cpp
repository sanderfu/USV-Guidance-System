#include "usv_simulator/sim_land.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"land");
    ros::NodeHandle land_nh("land");

    SimulatedLand sim_land(land_nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration rate(1);
    while (ros::ok())
    {
        rate.sleep();
    }
    ros::shutdown();
}