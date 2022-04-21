#include "usv_simulator/monte_carlo_supervisor.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"monte_carlo_supervisor");
    ros::NodeHandle nh;
    MonteCarloSupervisor mc_supervisor(nh);
    mc_supervisor.runSimulations();
    ros::spin();
}