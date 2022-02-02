#include "usv_simulator/sim_tracker.h"

SimulatedTracker::SimulatedTracker(ros::NodeHandle& nh) : nh_(nh){
    
    nh_.getParam("id",id_);
    std::cout << id_ << std::endl;
    odom_sub_ = nh_.subscribe("odom",1,&SimulatedTracker::odomCb,this);
    obstacle_pub_ = nh_.advertise<usv_simulator::obstacle>("/obstacles",1,false);
    main_loop_timer_ = nh_.createTimer(ros::Duration(1),&SimulatedTracker::mainLoop,this);
}

void SimulatedTracker::odomCb(const nav_msgs::Odometry& msg){
    latest_odom_=msg;
}

void SimulatedTracker::mainLoop(const ros::TimerEvent& e){
    usv_simulator::obstacle obst;
    obst.odom = latest_odom_;
    obst.id = id_;
    obstacle_pub_.publish(obst);
}