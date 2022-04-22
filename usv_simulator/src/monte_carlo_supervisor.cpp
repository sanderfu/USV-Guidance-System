#include "usv_simulator/monte_carlo_supervisor.h"

MonteCarloSupervisor::MonteCarloSupervisor(const ros::NodeHandle& nh): nh_(nh){
    //Register publishers
    system_reinit_pub_ = nh_.advertise<usv_msgs::reinit>("mc/system_reinit",10,true);

    if(!nh_.getParam("initial_position",global_position_vec_)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
    }
    tf::Quaternion q;
    q.setRPY(0,0,global_position_vec_[2]);

    //Monte Carlo parameters
    bool parameter_load_error = false;
    if(!ros::param::get("monte_carlo_supervisor/leader",leader_supervisor_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/simulation_collection_name",simulation_collection_name_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/simulations",simulations_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/predefined_mission",predefined_mission_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/predefined_mission_name",predefined_mission_name_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/goal_pose",goal_pose_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }

    //If leader, advertise to leader topics
    if(leader_supervisor_){
        leader_done_pub_ = nh_.advertise<std_msgs::Bool>("/monte_carlo_leader_supervisor/done",10);
    }
}

void MonteCarloSupervisor::runSimulations(){
    int sim_id = 0;
    while(sim_id<simulations_){
        //Reinit all actual parts of guidance system
        usv_msgs::reinit reinit_msg;
        reinit_msg.header.stamp = ros::Time::now();
        reinit_msg.mission_name.data = simulation_collection_name_ + std::to_string(sim_id);
        reinit_msg.initial_pose.position.x = global_position_vec_[0];
        reinit_msg.initial_pose.position.y = global_position_vec_[1];

        tf::Quaternion q;
        q.setRPY(0,0,global_position_vec_[2]);
        tf::quaternionTFToMsg(q,reinit_msg.initial_pose.orientation);

        system_reinit_pub_.publish(reinit_msg);

        //Send mission to mission planner (blocks until search done)
        sendMission(simulation_collection_name_+std::to_string(sim_id));

        if(leader_supervisor_){
            ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/done");
            leader_done_pub_.publish(std_msgs::Bool());
        } else{
            ros::topic::waitForMessage<std_msgs::Bool>("/monte_carlo_leader_supervisor/done");
        }
        sim_id++;
    }
    ros::Duration(2.0).sleep();
    ros::shutdown();

}

void MonteCarloSupervisor::sendMission(std::string mission_name){
    MissionPlannerClient mission_planner_client(nh_);
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh_);
    ros::Duration(2).sleep();
    if(!predefined_mission_){
        mission_planner_client.searchFromOdom(goal_pose_[0],goal_pose_[1],mission_name);
    } else{
        mission_planner_client.loadPredefined(predefined_mission_name_);
    }
}