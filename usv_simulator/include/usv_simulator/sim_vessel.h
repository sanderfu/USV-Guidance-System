#pragma once

#include "usv_model/model_library.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"

#include <chrono>
/**
 * @brief A general ros wrapper for surface vessel models
 * 
 */
class SimulatedVessel{
    public:
        SimulatedVessel(const ros::NodeHandle& nh, state_type& x_init);
    private:
        state_type x_;
        ModelLibrary::Viknes830 vessel_model_;
        std::string vessel_name_;

        double update_frequency_;

        ros::NodeHandle nh_;
        tf::TransformBroadcaster tf_broad_;
        ros::Publisher pose_pub_;
        ros::Publisher odom_pub_;
        ros::Publisher noise_pub_;
        ros::Subscriber cmd_sub_;
        ros::Timer loop_timer_;

        double u_d_;
        double psi_d_;
        double r_d_;

        void cmdCb(const geometry_msgs::Twist& msg);
        void updateLoop(const ros::TimerEvent& e);
        void publishData();
        void start();


};