#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "usv_msgs/reinit.h"
#include "tf/transform_broadcaster.h"
#include "usv_mission_planner/mission_planner.h"

class MonteCarloSupervisor{
    public:
        MonteCarloSupervisor(const ros::NodeHandle& nh);
        void runSimulations();
    private:
        ros::NodeHandle nh_;
        ros::Publisher vessel_reinit_pub_;
        ros::Publisher system_reinit_pub_;

        std::vector<double> global_position_vec_; //The position around which perturbations are made

        void sendMission(std::string mission_name);
};