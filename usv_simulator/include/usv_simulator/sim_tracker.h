#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "usv_simulator/obstacle.h"
#include <string>

class SimulatedTracker{
    public:
        SimulatedTracker(ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        ros::Publisher obstacle_pub_;
        ros::Timer main_loop_timer_;

        int id_;
        nav_msgs::Odometry latest_odom_;

        void odomCb(const nav_msgs::Odometry& msg);
        void mainLoop(const ros::TimerEvent& e);
};