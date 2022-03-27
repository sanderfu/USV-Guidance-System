#pragma once

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "gdal/ogrsf_frmts.h"
#include "usv_map/map_service.h"
#include "usv_map/geotf_ros.h"
#include "usv_model/model_library.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "geotf/geodetic_converter.h"
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

class SimulationBasedMPC{
    public:
        SimulationBasedMPC(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber dynamic_obstacles_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber los_setpoint_sub_;
        ros::Subscriber obstacle_sub_;
        ros::Publisher correction_pub_;
        ros::Timer main_loop_timer_;
        MapService* map_service_;

        geotf::GeodeticConverter geo_converter_;

        ModelLibrary::Viknes830 usv_;
        nav_msgs::Odometry latest_odom_;
        geometry_msgs::Twist latest_los_setpoint_;


        void odomCb(const nav_msgs::Odometry& odom);
        void losSetpointSub(const geometry_msgs::Twist& msg);
        void obstacleCb(const usv_simulator::obstacle& msg);
        void mainLoop(const ros::TimerEvent& e);

        void getBestControlOffset(double& u_d_best, double& psi_d_best);
        double costFnc(ModelLibrary::simulatedHorizon& usv_horizon, ModelLibrary::simulatedHorizon& obstacle_horizon, double P_ca, double Chi_ca, int k);
        double Delta_P(double P_ca);
        double Delta_Chi(double Chi_ca);

        std::vector<double> Chi_ca_;
		std::vector<double> P_ca_;
        std::map<int,obstacleVessel*> obstacle_vessels_;
      
        double Chi_ca_last_;
		double P_ca_last_;

        // Cost function weights;
		const double P_;
		const double Q_;
		const double D_CLOSE_;
		const double D_SAFE_;
		const double K_COLL_;
		const double PHI_AH_;
		const double PHI_OT_;
		const double PHI_HO_;
		const double PHI_CR_;
		const double KAPPA_;
		const double K_P_;
		const double K_CHI_;
		const double K_DP_;
		const double K_DCHI_SB_;
		const double K_DCHI_P_;
		
		const double T_; 	// Prediction horizon
        const double DT_;	// Step length, obstacle trajectory prediction 		
		
		int n_samp;

        //Visualization (for debug purposes)
        ros::Publisher path_viz_pub_;
        visualization_msgs::Marker path_viz_;
        void visualizePath(OGRLineString& path);
        void clearVisualPath();
};