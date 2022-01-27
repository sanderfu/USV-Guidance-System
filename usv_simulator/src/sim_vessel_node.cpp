#include "usv_simulator/sim_vessel.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"sim");
    ros::NodeHandle vessel_nh;

    state_type x_init(6);
    x_init[0] = 0;
    x_init[1] = 0;
    x_init[2] = 0;
    x_init[3] = 0;
    x_init[4] = 0;
    x_init[5] = 0;
    SimulatedVessel vessel1(vessel_nh,x_init);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration rate(1);
    while (ros::ok())
    {
        rate.sleep();
    }
    ros::shutdown();
}