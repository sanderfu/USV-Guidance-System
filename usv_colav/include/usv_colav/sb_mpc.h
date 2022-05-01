#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "usv_msgs/reinit.h"
#include "usv_msgs/Colav.h"
#include "usv_msgs/ColavPath.h"
#include "std_msgs/Bool.h"
#include "gdal/ogrsf_frmts.h"
#include "usv_map/map_service.h"
#include "usv_model/model_library.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "geotf/geodetic_converter.h"
#include "usv_map/geotf_sync.h"
#include "usv_simulator/obstacle.h"
#include "map"

static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = 180.0f/M_PI;

void rot2d(double yaw, Eigen::Vector2d &res);

struct obstacleVessel{
    obstacleVessel(state_type init,double length, double width) : model_(length, width), latest_obstacle_state_(init), latest_observation_(ros::Time::now()){}
    int id;
    ros::Time latest_observation_;
    ModelLibrary::LinearObstacleShip model_;
    state_type latest_obstacle_state_;
};

struct controlCandidate{
    controlCandidate(double p_cand, double chi_cand, ModelLibrary::simulatedHorizon horizon,double cost):p_cand_(p_cand),chi_cand_(chi_cand),horizon_(horizon),cost_(cost){};
    double p_cand_;
    double chi_cand_;
    ModelLibrary::simulatedHorizon horizon_;
    double cost_;
};

class SimulationBasedMPC{
    public:
        SimulationBasedMPC(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber dynamic_obstacles_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber los_setpoint_sub_;
        ros::Subscriber obstacle_sub_;
        ros::Subscriber system_reinit_sub_;
        ros::Publisher correction_pub_;
        ros::Timer main_loop_timer_;
        MapService* map_service_;
    
        geotf::GeodeticConverterSynchronized geo_converter_;

        ModelLibrary::Viknes830 usv_;
        nav_msgs::Odometry latest_odom_;
        geometry_msgs::Twist latest_los_setpoint_;


        void odomCb(const nav_msgs::Odometry& odom);
        void losSetpointSub(const geometry_msgs::Twist& msg);
        void obstacleCb(const usv_simulator::obstacle& msg);
        void mainLoop(const ros::TimerEvent& e);
        void reinitCb(const usv_msgs::reinit& msg);

        void getBestControlOffset(double& u_d_best, double& psi_d_best);
        double costFnc(ModelLibrary::simulatedHorizon& usv_horizon, obstacleVessel& obstacle_vessel, double P_ca, double Chi_ca, int k, double t_offset, double P_ca_last, double Chi_ca_last);
        double Delta_P(double P_ca, double P_ca_last);
        double Delta_Chi(double Chi_ca, double Chi_ca_last);

        std::vector<double> Chi_ca_;
		std::vector<double> P_ca_;

        //WIP Control action pairs
        std::vector<std::vector<double>> Chi_ca_sequences_;

        std::map<int,obstacleVessel*> obstacle_vessels_;

        std::map<double,controlCandidate> control_candidate_map;
      
        double Chi_ca_last_;
		double P_ca_last_;

        // Cost function weights;
		double P_;
		double Q_;
		double D_CLOSE_;
		double D_SAFE_;
		double K_COLL_;
		double PHI_AH_;
		double PHI_OT_;
		double PHI_HO_;
		double PHI_CR_;
		double KAPPA_;
		double K_P_;
		double K_CHI_;
		double K_DP_;
		double K_DCHI_SB_;
		double K_DCHI_P_;

        int ownship_id_;
        double update_frequency_;
        double prediction_time_;
        bool verbose_;

        //Visualization (for debug purposes)
        ros::Publisher path_viz_pub_;
        ros::Publisher colav_data_pub_;
        visualization_msgs::Marker path_viz_;
        usv_msgs::Colav colav_msg_;
        void visualizePath(OGRLineString& path);
        void clearVisualPath();
};