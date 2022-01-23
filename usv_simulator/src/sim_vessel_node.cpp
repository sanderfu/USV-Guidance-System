#include "usv_simulator/sim_vessel.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"vessel1");
    ros::NodeHandle vessel_nh("vessel1");
    ROS_INFO_STREAM("USV sleeping for 5 seconds before starting");
    ros::Duration(5).sleep();

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