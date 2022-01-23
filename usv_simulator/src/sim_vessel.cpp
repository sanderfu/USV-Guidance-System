# include "usv_simulator/sim_vessel.h"

SimulatedVessel::SimulatedVessel(const ros::NodeHandle& nh, state_type& x_init ) : nh_(nh){

    x_ = x_init;
    update_frequency_ = 10;
    vessel_name_ = ros::names::clean(nh_.getNamespace());

    // Setup transform broadcaster
    tf_broad_ = tf::TransformBroadcaster();

    // Setup publishers and subscribers 
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("state", 10);
    noise_pub_ = nh_.advertise<geometry_msgs::Vector3>("wave_noise", 10);
    cmd_sub_ = nh_.subscribe("cmd_vel", 1, &SimulatedVessel::cmdCb, this);

    //Set up driving timer
    loop_timer_ = nh_.createTimer(ros::Duration(1/update_frequency_),&SimulatedVessel::updateLoop,this);
    
}

void SimulatedVessel::updateLoop(const ros::TimerEvent& e){

    ros::Time loop_time_start = ros::Time::now();
    vessel_model_.simulate(x_,u_d_, psi_d_,1/update_frequency_);
    publishData();
    ros::Time loop_time_end = ros::Time::now();

    //Verify that loop is not lagging behind it's intended frequency
    ROS_WARN_STREAM_COND_NAMED((ros::Duration(loop_time_end-loop_time_start).toSec()>1/update_frequency_),"UpdateLoopTiming","The update loop is falling behind");

}

void SimulatedVessel::publishData(){
    tf::Transform transform;
    nav_msgs::Odometry odom;

    transform.setOrigin(tf::Vector3(x_[0],x_[1],0));

    tf::Quaternion q;
    q.setRPY(0,0,x_[2]);
    transform.setRotation(q);

    tf_broad_.sendTransform(tf::StampedTransform(transform,
                                            ros::Time::now(),
                                            "map",
                                            nh_.getNamespace()));

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
}

void SimulatedVessel::cmdCb(const geometry_msgs::Twist& msg){
    u_d_ = msg.linear.x;
    psi_d_ = msg.angular.y;
}
