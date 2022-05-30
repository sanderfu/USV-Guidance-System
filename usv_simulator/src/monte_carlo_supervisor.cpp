#include "usv_simulator/monte_carlo_supervisor.h"

MonteCarloSupervisor::MonteCarloSupervisor(const ros::NodeHandle& nh): nh_(nh), noise_generator_ptr_(new std::default_random_engine(std::random_device()())){
    //Register publishers
    system_reinit_pub_ = nh_.advertise<usv_msgs::reinit>("mc/system_reinit",10,true);

    //Monte Carlo parameters
    bool parameter_load_error = false;
    if(!ros::param::get("monte_carlo_supervisor/leader",leader_supervisor_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/initial_pose_variance/enabled",initial_pose_variance_enabled_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/initial_pose_variance/x",initial_pose_variance_x_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/initial_pose_variance/y",initial_pose_variance_y_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/initial_pose_variance/course",initial_pose_variance_course_)) parameter_load_error = true;
    if(!ros::param::get("monte_carlo_supervisor/done_timeout_time",done_timeout_time_)) parameter_load_error = true;
    if(!ros::param::get("initial_pose",global_position_vec_)) parameter_load_error = true;
    if(!ros::param::get("simulation_collection_name",simulation_collection_name_)) parameter_load_error = true;
    if(!ros::param::get("simulations",simulations_)) parameter_load_error = true;
    if(!ros::param::get("predefined_mission",predefined_mission_)) parameter_load_error = true;
    if(!ros::param::get("predefined_mission_name",predefined_mission_name_)) parameter_load_error = true;
    if(!ros::param::get("goal_pose",goal_pose_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }

    //If leader, advertise to leader topics
    if(leader_supervisor_){
        leader_done_pub_ = nh_.advertise<std_msgs::Bool>("/monte_carlo_leader_supervisor/done",10);
    }

    //Set pose noise distributions
    noise_distribution_x_ = std::normal_distribution<double>(0,initial_pose_variance_x_);
    noise_distribution_y_ = std::normal_distribution<double>(0,initial_pose_variance_y_);
    noise_distribution_course_ = std::normal_distribution<double>(0,initial_pose_variance_course_);

}

void MonteCarloSupervisor::runSimulations(){
    int sim_id = 0;
    while(sim_id<simulations_){
        //Reinit all actual parts of guidance system
        usv_msgs::reinit reinit_msg;
        reinit_msg.header.stamp = ros::Time::now();
        reinit_msg.mission_name.data = simulation_collection_name_ + std::to_string(sim_id);

        reinit_msg.initial_pose.position.x = global_position_vec_[0]+initial_pose_variance_enabled_*noise_distribution_x_(*noise_generator_ptr_);
        reinit_msg.initial_pose.position.y = global_position_vec_[1]+initial_pose_variance_enabled_*noise_distribution_y_(*noise_generator_ptr_);

        tf::Quaternion q;
        double course_noise = initial_pose_variance_enabled_*noise_distribution_course_(*noise_generator_ptr_);
        ROS_WARN_STREAM(nh_.getNamespace() << " course noise: " << course_noise);
        ROS_WARN_STREAM(nh_.getNamespace() << " initial course: " << global_position_vec_[2]+course_noise);

        q.setRPY(0,0,global_position_vec_[2]+course_noise);
        tf::quaternionTFToMsg(q,reinit_msg.initial_pose.orientation);

        system_reinit_pub_.publish(reinit_msg);

        //Send mission to mission planner (blocks until search done)
        sendMission(simulation_collection_name_+std::to_string(sim_id));

        if(leader_supervisor_){
            std_msgs::BoolConstPtr msg = ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/done",ros::Duration(done_timeout_time_));
            std_msgs::Bool done_msg;
            done_msg.data = true;
            if(msg==NULL) done_msg.data=false;
            leader_done_pub_.publish(done_msg);
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
        mission_planner_client.searchFromOdom(goal_pose_[0],goal_pose_[1],goal_pose_[2],mission_name);
    } else{
        mission_planner_client.loadPredefined(predefined_mission_name_);
    }
}