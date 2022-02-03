#include "usv_simulator/sim_vessel.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"sim");
    ros::NodeHandle vessel_nh;

    
    SimulatedVessel vessel1(vessel_nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration rate(1);
    while (ros::ok())
    {
        rate.sleep();
    }
    ros::shutdown();
}