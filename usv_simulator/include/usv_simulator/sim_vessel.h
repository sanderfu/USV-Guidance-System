#pragma once

#include "usv_model/model_library.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "usv_msgs/reinit.h"
#include "nav_msgs/Odometry.h"
#include "geotf/geodetic_converter.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "vector"

#include <chrono>
/**
 * @brief A general ros wrapper for surface vessel models
 * 
 */
class SimulatedVessel{
    public:
        SimulatedVessel(const ros::NodeHandle& nh);
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

        geotf::GeodeticConverter converter_;

        double u_d_;
        double psi_d_;
        double r_d_;

        void cmdCb(const geometry_msgs::Twist& msg);
        void updateLoop(const ros::TimerEvent& e);
        void publishData();
        void start();

        //For Monte Carlo Simulations
        ros::Subscriber reinit_state_sub_; //Pose subscriber, if called the vessel is set to this state
        void reinitCb(const usv_msgs::reinit msg);


};