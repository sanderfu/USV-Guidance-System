#include "usv_simulator/sim_tracker.h"



int main(int argc, char** argv){
    ros::init(argc,argv,"obstacle_tracker");
    ros::NodeHandle nh;

    SimulatedTracker tracker(nh);
    ros::spin();
}