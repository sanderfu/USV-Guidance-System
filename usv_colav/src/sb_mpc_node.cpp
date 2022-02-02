#include "usv_colav/sb_mpc.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"sp_mpc_node");
    ros::NodeHandle nh;

    SimulationBasedMPC mpc(nh);
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

}