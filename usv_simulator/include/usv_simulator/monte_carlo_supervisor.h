#include "ros/ros.h"
#include "ros/topic.h"
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
        ros::Publisher leader_done_pub_; //Only the supervisor set as "leader" registers for publishing to this topic

        //Parameters
        bool leader_supervisor_;
        std::string simulation_collection_name_; 
        int simulations_;
        bool predefined_mission_;
        std::string predefined_mission_name_;
        std::vector<double> goal_pose_;
        std::vector<double> global_position_vec_; //The position around which perturbations are made
        
        void sendMission(std::string mission_name);
};