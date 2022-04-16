#include "usv_simulator/monte_carlo_supervisor.h"

MonteCarloSupervisor::MonteCarloSupervisor(const ros::NodeHandle& nh): nh_(nh){
    //Register publishers
    vessel_reinit_pub_ = nh_.advertise<nav_msgs::Odometry>("mc/reinit_vessel_state",10);
    system_reinit_pub_ = nh_.advertise<usv_msgs::reinit>("mc/system_reinit",10);

    if(!nh_.getParam("initial_position",global_position_vec_)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
    }
    tf::Quaternion q;
    q.setRPY(0,0,global_position_vec_[2]);

    //Monte Carlo parameters

}

void MonteCarloSupervisor::runSimulations(){
    std::string simulation_name = "test_mc_";
    int sim_id = 0;
    while(true){
        //Reinit all actual parts of guidance system
        usv_msgs::reinit reinit_msg;
        reinit_msg.header.stamp = ros::Time::now();
        reinit_msg.mission_name.data = simulation_name + std::to_string(sim_id);
        reinit_msg.initial_pose.position.x = global_position_vec_[0];
        reinit_msg.initial_pose.position.y = global_position_vec_[1];

        tf::Quaternion q;
        q.setRPY(0,0,global_position_vec_[2]);
        tf::quaternionTFToMsg(q,reinit_msg.initial_pose.orientation);

        system_reinit_pub_.publish(reinit_msg);

        //Send mission to mission planner (blocks until search done)
        std::cout << "Sending mission, blocking until done" << std::endl;
        sendMission(simulation_name+std::to_string(sim_id));
        std::cout << "Mission done" << std::endl;

        std::cout << "Waiting 10s and starting over" << std::endl;
        ros::Duration(10.0).sleep();
        sim_id++;
    }

}

void MonteCarloSupervisor::sendMission(std::string mission_name){
    MissionPlannerClient mission_planner_client(nh_);
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh_);
    ros::Duration(2).sleep();
    mission_planner_client.searchFromOdom(-73.839272,40.641694,mission_name);
}