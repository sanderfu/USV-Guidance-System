#include "usv_colav/sb_mpc.h"

SimulationBasedMPC::SimulationBasedMPC(const ros::NodeHandle& nh) : 
nh_(nh),
map_client_(&nh_),
geodetic_client_(&nh_),
T_(300.0), 				// 150.0  //300
DT_(0.5), 				//   0.05 // 0.5
P_(1), 					//   1.0
Q_(4.0), 				//   4.0
D_CLOSE_(200.0),		// 200.0
D_SAFE_(40.0),			//  40.0
K_COLL_(0.5),			//   0.5
PHI_AH_(15.0),			//  15.0
PHI_OT_(68.5),			//  68.5
PHI_HO_(22.5),			//  22.5
PHI_CR_(68.5),			//  68.0
KAPPA_(2.0),			//   3.0
K_P_(4.0),				//   2.5  // 4.0
K_CHI_(1.3),			//   1.3
K_DP_(3.5),				//   2.0  // 3.5
K_DCHI_SB_(0.9),		//   0.9
K_DCHI_P_(1.2)			//   1.2
{
    ROS_INFO_STREAM("SB_MPC started, waiting for first odometry from USV");
    latest_odom_ = *ros::topic::waitForMessage<nav_msgs::Odometry>("odom",nh_);
    ROS_INFO_STREAM("Odometry received");

    odom_sub_ = nh_.subscribe("odom",1,&SimulationBasedMPC::odomCb,this);
    los_setpoint_sub_ = nh_.subscribe("los/setpoint",1,&SimulationBasedMPC::losSetpointSub,this);
    main_loop_timer_ = nh_.createTimer(ros::Duration(1),&SimulationBasedMPC::mainLoop,this);

    //Set course action alternatives
    double courseOffsets[] = {-90.0,-75.0,-60.0,-45.0,-30.0,-15.0,0.0,15.0,30.0,45.0,60.0,75.0,90.0};
    double sizeCO = sizeof(courseOffsets)/sizeof(courseOffsets[0]);
    for (int i = 0; i < sizeCO; i++){
    	courseOffsets[i] *= DEG2RAD;
    }
    Chi_ca_.assign(courseOffsets, courseOffsets + sizeof(courseOffsets)/sizeof(courseOffsets[0]));

    //Set speed action alternatives
    double speedOffsets[] = {-1,0,0.5,1};
    P_ca_.assign(speedOffsets, speedOffsets + sizeof(speedOffsets)/sizeof(speedOffsets[0]));

}

void SimulationBasedMPC::odomCb(const nav_msgs::Odometry& odom){
    latest_odom_ = odom;
}

void SimulationBasedMPC::losSetpointSub(const geometry_msgs::Twist& msg){
    latest_los_setpoint_ = msg;
}

 void mainLoop(const ros::TimerEvent& e){
    std::cout << "Hello world" << std::endl;
}