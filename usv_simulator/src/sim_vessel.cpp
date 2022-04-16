# include "usv_simulator/sim_vessel.h"

SimulatedVessel::SimulatedVessel(const ros::NodeHandle& nh) : nh_(nh), geo_converter_(&nh_), x_(6){
    std::vector<double> global_position_vec;
    if(!nh_.getParam("initial_position",global_position_vec)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
    }
    std::cout <<"global position vec: " << global_position_vec[0] << " " << global_position_vec[1] << std::endl;
    Eigen::Vector3d global_initial_position(global_position_vec[0],global_position_vec[1],0);
    Eigen::Vector3d local_initial_position;
    while(!geo_converter_.convert("WGS84",global_initial_position,"global_enu",local_initial_position));
    x_[0] = local_initial_position(0);
    x_[1] = local_initial_position(1);
    x_[2] = global_position_vec[2];
    x_[3] = 0;
    x_[4] = 0;
    x_[5] = 0;
    update_frequency_ = 30;
    vessel_name_ = ros::this_node::getNamespace().substr(1,ros::this_node::getNamespace().size()-1);
    u_d_ = 0;
    psi_d_ = 0; 

    // Setup transform broadcaster
    tf_broad_ = tf::TransformBroadcaster();

    // Setup publishers and subscribers 
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    noise_pub_ = nh_.advertise<geometry_msgs::Vector3>("wave_noise", 10);
    cmd_sub_ = nh_.subscribe("los/corrected_setpoint", 1, &SimulatedVessel::cmdCb, this);

    //Set up driving timer
    loop_timer_ = nh_.createTimer(ros::Duration(1/update_frequency_),&SimulatedVessel::updateLoop,this);

    //Setup Monte Carlo Compatebility
    reinit_state_sub_ = nh_.subscribe("mc/system_reinit",1,&SimulatedVessel::reinitCb,this);
    
}
/**
 * @brief The main simulation loop
 * 
 * @param e 
 */
void SimulatedVessel::updateLoop(const ros::TimerEvent& e){

    ros::Time loop_time_start = ros::Time::now();
    vessel_model_.simulate(x_,u_d_, psi_d_,1/update_frequency_);
    publishData();
    ros::Time loop_time_end = ros::Time::now();

    //Verify that loop is not lagging behind it's intended frequency
    ROS_WARN_STREAM_COND_NAMED((ros::Duration(loop_time_end-loop_time_start).toSec()>1/update_frequency_),"UpdateLoopTiming","The update loop is falling behind");

}

/**
 * @brief Publish both the frame transformation and vessel odometry
 * 
 */
void SimulatedVessel::publishData(){
    tf::Transform transform;
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped pose;

    //Transform publish
    transform.setOrigin(tf::Vector3(x_[0],x_[1],0));

    tf::Quaternion q;
    q.setRPY(0,0,x_[2]);
    transform.setRotation(q);

    tf_broad_.sendTransform(tf::StampedTransform(transform,
                                            ros::Time::now(),
                                            "map",
                                            vessel_name_));

    //Odometry publish (local frame)
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = nh_.getNamespace();

    odom.pose.pose.position.x = x_[0];
    odom.pose.pose.position.y = x_[1];

    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

    odom.twist.twist.linear.x = x_[3];
    odom.twist.twist.linear.y = x_[4];
    odom.twist.twist.angular.z = x_[5];
    odom_pub_.publish(odom);

    //Pose publish (global frame, LLA)
    Eigen::Vector3d local_initial_position(x_[0],x_[1],0);
    Eigen::Vector3d global_initial_position;
    while(!geo_converter_.convert("global_enu",local_initial_position,"WGS84",global_initial_position));
    pose.pose.position.x = global_initial_position.x();
    pose.pose.position.y = global_initial_position.y();
    tf::quaternionTFToMsg(q, pose.pose.orientation);
    pose_pub_.publish(pose);


}

/**
 * @brief Callback for receiving desired velocity and heading(or course?)
 * 
 * @param msg Callback message containing desired setpoints
 */
void SimulatedVessel::cmdCb(const geometry_msgs::Twist& msg){
    u_d_ = msg.linear.x;
    psi_d_ = msg.angular.z;
}

void SimulatedVessel::reinitCb(const usv_msgs::reinit msg){
    //Stop update timer
    loop_timer_.stop();

    Eigen::Vector3d global_initial_position(msg.initial_pose.position.x,msg.initial_pose.position.y,0);
    Eigen::Vector3d local_initial_position;
    while(!geo_converter_.convert("WGS84",global_initial_position,"global_enu",local_initial_position));

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.initial_pose.orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch,yaw;
    m.getRPY(roll, pitch, yaw);

    x_[0] = local_initial_position(0);
    x_[1] = local_initial_position(1);
    x_[2] = yaw;
    x_[3] = 0;
    x_[4] = 0;
    x_[5] = 0;

    //Reinit done, start update timer again
    loop_timer_.start();
}


