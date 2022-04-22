#include "usv_colav/sb_mpc.h"

SimulationBasedMPC::SimulationBasedMPC(const ros::NodeHandle& nh) : 
nh_(nh),
T_(300.0), 				// 150.0  //300
DT_(0.5), 				//   0.05 // 0.5
P_(1), 					//   1.0
Q_(4.0), 				//   4.0
D_CLOSE_(200.0),		// 200.0
D_SAFE_(40.0),			//  40.0
K_COLL_(0.5),			//   0.5
PHI_AH_(15.0),			//  15.0
PHI_OT_(68.5),			//  68.5
PHI_HO_(22.5),			//  22.5
PHI_CR_(68.5),			//  68.0
KAPPA_(2.0),			//   3.0
K_P_(4.0),				//   2.5  // 4.0
K_CHI_(1.3),			//   1.3
K_DP_(3.5),				//   2.0  // 3.5
K_DCHI_SB_(0.9),		//   0.9
K_DCHI_P_(1.2),			//   1.2
geo_converter_("COLAV",false,true,nh)
{   
    bool parameter_load_error = false;
    std::string map_name;
    if(!ros::param::get("mission_planner/map_name",map_name)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
    map_service_ = new MapService(map_name);
    //Warning: I do not know if this will onlky surpress warnings for GDAL related to this module or also the mission planner. 
    ROS_WARN_STREAM("GDAL errors are supressed!");
    CPLPushErrorHandler(CPLQuietErrorHandler);
    
    ROS_INFO_STREAM("SB_MPC started, waiting for first odometry and LOS setpoint from USV");
    latest_odom_ = *ros::topic::waitForMessage<nav_msgs::Odometry>("odom",nh_);
    latest_los_setpoint_ = *ros::topic::waitForMessage<geometry_msgs::Twist>("los/setpoint",nh_);
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh_);
    ROS_INFO_STREAM("Odometry and LOS setpoint received");

    std::vector<double> global_position_vec;
    if(!nh_.getParam("initial_position",global_position_vec)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
    }

    correction_pub_ = nh_.advertise<geometry_msgs::Twist>("colav/correction",1,false);
    colav_data_pub_ = nh_.advertise<usv_msgs::Colav>("colav/debug",1,false);

    odom_sub_ = nh_.subscribe("odom",1,&SimulationBasedMPC::odomCb,this);
    los_setpoint_sub_ = nh_.subscribe("los/setpoint",1,&SimulationBasedMPC::losSetpointSub,this);
    obstacle_sub_ = nh_.subscribe("/obstacles",1,&SimulationBasedMPC::obstacleCb,this);
    system_reinit_sub_ = nh_.subscribe("mc/system_reinit",1,&SimulationBasedMPC::reinitCb,this);
    main_loop_timer_ = nh_.createTimer(ros::Duration(2.5),&SimulationBasedMPC::mainLoop,this);

    //Set course action alternatives
    double courseOffsets[] = {89.0,-75.0,-60.0,-45.0,-30.0,-15.0,0.0,15.0,30.0,45.0,60.0,75.0,89.0};
    double sizeCO = sizeof(courseOffsets)/sizeof(courseOffsets[0]);
    for (int i = 0; i < sizeCO; i++){
    	courseOffsets[i] *= DEG2RAD;
    }
    Chi_ca_.assign(courseOffsets, courseOffsets + sizeof(courseOffsets)/sizeof(courseOffsets[0]));

    //Set speed action alternatives
    double speedOffsets[] = {-1,0,0.5,1};
    P_ca_.assign(speedOffsets, speedOffsets + sizeof(speedOffsets)/sizeof(speedOffsets[0]));
    Chi_ca_last_ = 0.0;
    P_ca_last_ = 1.0;

    //Initialize visualization stuff (debug purposes)
    path_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("colav/path_viz",1,true);
    path_viz_.header.frame_id="map";
    path_viz_.header.stamp = ros::Time::now();
    path_viz_.action = visualization_msgs::Marker::ADD;
    path_viz_.type = visualization_msgs::Marker::LINE_LIST;
    path_viz_.lifetime = ros::Duration();
    // Set the color -- be sure to set alpha to something non-zero!
    path_viz_.color.r = 1.0f;
    path_viz_.color.g = 0.0f;
    path_viz_.color.b = 0.0f;
    path_viz_.color.a = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    path_viz_.scale.x = 2.5;

}
/**
 * @brief Callback when receiving odometry from vessel on which the SBMPC is running.
 * 
 * @param odom Odometry message
 */
void SimulationBasedMPC::odomCb(const nav_msgs::Odometry& odom){
    latest_odom_ = odom;
}

/**
 * @brief Callback when receiving LOS setpoints from vessel on which the SBMPC is running.
 * 
 * @param msg Twist message (linear.x=speed setpoint, angular.z=yaw setpoint)
 */
void SimulationBasedMPC::losSetpointSub(const geometry_msgs::Twist& msg){
    latest_los_setpoint_ = msg;
}

/**
 * @brief Callback when receiving obstacle from obstacle tracking system on vessel on which the SBMPC is running
 * 
 * @param msg Obstacle message (custom message type)
 */
void SimulationBasedMPC::obstacleCb(const usv_simulator::obstacle& msg){
    state_type state(6);
    Eigen::Vector3d position_wgs(msg.odom.pose.pose.position.x,msg.odom.pose.pose.position.y,msg.odom.pose.pose.position.z);
    Eigen::Vector3d point_enu;
    geo_converter_.convertSynced("WGS84",position_wgs,"global_enu",&point_enu);
    state[0]=point_enu.x();
    state[1]=point_enu.y();
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.odom.pose.pose.orientation,q);
    tf::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll,pitch,yaw);
    state[2]=yaw;
    state[3]=msg.odom.twist.twist.linear.x;
    state[4]=msg.odom.twist.twist.linear.y;
    state[5]=msg.odom.twist.twist.angular.z;
    if(obstacle_vessels_.find(msg.id)==obstacle_vessels_.end()){
        obstacle_vessels_[msg.id] = new obstacleVessel(state,5,2);
    }else{
        obstacle_vessels_[msg.id]->latest_obstacle_state_ = state;
        obstacle_vessels_[msg.id]->latest_observation_ = ros::Time::now();
    }
}

/**
 * @brief Calculate the lowest-cost control offsets for speed and yaw.
 * 
 * @param u_corr_best Best speed correction (multiplier) (-1,0,0.5,1)
 * @param psi_corr_best Best yaw correction  
 */
void SimulationBasedMPC::getBestControlOffset(double& u_corr_best, double& psi_corr_best){
    double cost = INFINITY;
    double cost_k = 0.0;
    tfScalar roll, pitch,yaw;
    tf::Quaternion q;
    Eigen::Vector3d point_local;
    Eigen::Vector3d point_global;
    OGRLineString choosen_path;

    //Clear last path options and cost options
    colav_msg_.path_options.clear();
    colav_msg_.cost_options.clear();

    std::map<int,ModelLibrary::simulatedHorizon> obstacles_horizon;
    for(auto obst_it=obstacle_vessels_.begin(); obst_it!=obstacle_vessels_.end(); obst_it++){
        std::map<int,ModelLibrary::simulatedHorizon>::iterator it = obstacles_horizon.begin();
        state_type test = obst_it->second->latest_obstacle_state_;
        obst_it->second->model_.simulateHorizon(obst_it->second->latest_obstacle_state_,60);
        obstacles_horizon.insert(it, std::pair<int,ModelLibrary::simulatedHorizon>(obst_it->first,obst_it->second->model_.simulateHorizon(obst_it->second->latest_obstacle_state_,60)));
    }
    state_type state(6);
    Eigen::Vector3d position_wgs(latest_odom_.pose.pose.position.x,latest_odom_.pose.pose.position.y,latest_odom_.pose.pose.position.z);
    Eigen::Vector3d point_enu;
    geo_converter_.convertSynced("WGS84",position_wgs,"global_enu",&point_enu);
    state[0] = point_enu.x();
    state[1] = point_enu.y();
    tf::quaternionMsgToTF(latest_odom_.pose.pose.orientation,q);
    tf::Matrix3x3 mat(q);
    mat.getRPY(roll,pitch,yaw);
    state[2] = yaw;
    state[3] = latest_odom_.twist.twist.linear.x;
    state[4] = latest_odom_.twist.twist.linear.y;
    state[5] = latest_odom_.twist.twist.angular.z;

    for(auto chi_it = Chi_ca_.begin(); chi_it!=Chi_ca_.end();chi_it++){
        for(auto p_it=P_ca_.begin(); p_it!=P_ca_.end();p_it++){
            double cost_i = 0.0;
            //Simulate vessel
            ModelLibrary::simulatedHorizon horizon = usv_.simulateHorizon(state,(latest_los_setpoint_.linear.x)*(*p_it),(latest_los_setpoint_.angular.z+(*chi_it)),60);
            
            //Avoid collision in COLREG compliant manner (if no vessels, penalize only the course and speed correction)
            if(obstacle_vessels_.size()!=0){
                for (auto it = obstacle_vessels_.begin(); it!=obstacle_vessels_.end();it++){
                    double key = it->first;
                    cost_k = costFnc(horizon, obstacles_horizon[key], *p_it, *chi_it, key);
                    if (cost_k > cost_i){
                        cost_i = cost_k;	// Maximizing cost associated with this scenario
                    }
			    }
            } else{
                cost_i = K_P_*(1-*p_it) + K_CHI_*pow(*chi_it,2) + Delta_P(*p_it) + Delta_Chi(*chi_it);
            }

            //Simulation horizon to LineString and add to data message
            OGRLineString path;
            usv_msgs::ColavPath msg_path;
            usv_msgs::ColavPathElement path_element;
            for(auto hor_it = horizon.state.begin(); hor_it!=horizon.state.end();hor_it++){
                point_local(0) = hor_it->at(0);
                point_local(1) = hor_it->at(1);
                point_local(2) = 0;
                geo_converter_.convertSynced("global_enu",point_local,"WGS84",&point_global);
                path.addPoint(point_global(0),point_global(1));

                //Add path to path options
                path_element.lon = hor_it->at(0);
                path_element.lat = hor_it->at(1);
                path_element.lon = hor_it->at(5);
                msg_path.path.push_back(path_element);
            }
            colav_msg_.path_options.push_back(msg_path);

            //Check path for collision
            if (map_service_->intersects(&path,LayerID::COLLISION)){
                cost_i += 100;
            }
            
            //TEMPORARY cost func, prioritize small manueuvers
            //cost_i += 0.55*abs((*chi_it+latest_los_setpoint_.angular.z)-yaw);
            //cost_i += abs(*chi_it/(M_PI/2));
            //cost_i += 10*abs(*p_it-1); 
            
            //Add cost option to msg
            colav_msg_.cost_options.push_back(cost_i);
            
            //Choose action based on minimized cost
            if (cost_i<cost){
                cost = cost_i;
                u_corr_best = *p_it;
                psi_corr_best = *chi_it; 
                choosen_path = path;

                //Update colav message
                colav_msg_.path = msg_path;
                colav_msg_.cost = cost;
                colav_msg_.speed_correction = u_corr_best;
                colav_msg_.course_correction = psi_corr_best;
            }

        }
    }
    P_ca_last_ = u_corr_best;
	Chi_ca_last_ = psi_corr_best;
    ROS_INFO_STREAM("Path num points: " << choosen_path.getNumPoints());
    ROS_INFO_STREAM("Best cost: " << cost);
    clearVisualPath();
    visualizePath(choosen_path);
}

/**
 * @brief The main loop of the SBMPC. 
 * 
 * @details Called periodically by designated timer. Calculates best offset from latest available information and pulishes this offset.
 * 
 * @param e TimerEveent parameter, currently not used.
 */
void SimulationBasedMPC::mainLoop(const ros::TimerEvent& e){
    double u_os, psi_os;
    ros::Time start = ros::Time::now();
    getBestControlOffset(u_os,psi_os);

    //Publish COLAV correction
    geometry_msgs::Twist offset;
    offset.linear.x = u_os;
    offset.angular.z = psi_os;
    correction_pub_.publish(offset);

    //Publish debug/post-visualization message
    colav_data_pub_.publish(colav_msg_);
    ros::Time stop = ros::Time::now();
    ROS_INFO_STREAM("Offset u_os: " << u_os << " Offset psi_os: " << psi_os*RAD2DEG);
    ROS_INFO_STREAM("Getting offset took: " << (stop-start).toSec() << " [s]");
}

void SimulationBasedMPC::reinitCb(const usv_msgs::reinit& msg){
    main_loop_timer_.stop();

    ROS_INFO_STREAM("COLAV reinit, waiting for odometry and LOS setpoint from USV");
    latest_odom_ = *ros::topic::waitForMessage<nav_msgs::Odometry>("odom",nh_);
    latest_los_setpoint_ = *ros::topic::waitForMessage<geometry_msgs::Twist>("los/setpoint",nh_);
    ROS_INFO_STREAM("Odometry and LOS setpoint received");
    
    main_loop_timer_.start();




}

/**
 * @brief Cost function to avoid other vessels in COLREG compliant manner. Calculate cost for a given obstacle vessel horizon.
 * 
 * @remark This cost function is from https://github.com/olesot/ros_asv_system
 * 
 * @param usv_horizon The simulated horizon for the ownship
 * @param obstacle_horizon The simulated horizon for an obstacle ship
 * @param P_ca Speed correction candidate giving simulated ownship horizon (multiplier)
 * @param Chi_ca Yaw correction giving simulated ownship horizon
 * @param k ID of obstacle ship
 * @return double Cost of action candidate.
 */
double SimulationBasedMPC::costFnc(ModelLibrary::simulatedHorizon& usv_horizon,ModelLibrary::simulatedHorizon& obstacle_horizon, double P_ca, double Chi_ca, int k)
{
	double dist, phi, psi_o, phi_o, psi_rel, R, C, k_coll, d_safe_i;
	Eigen::Vector2d d, los, los_inv, v_o, v_s;
	bool mu, OT, SB, HO, CR;
	double combined_radius = usv_.getL() + obstacle_vessels_[k]->model_.getL();
	double d_safe = D_SAFE_;
	double H0 = 0;
	double H1 = 0;
	double H2 = 0;
    double H3 = 0;
	double cost = 0;
	double t0 = 0;
    double t = 0;

	for (int i = 0; i < usv_horizon.steps; i++){

        t = usv_horizon.time[i];

		d(0) = obstacle_horizon.state[i][0] - usv_horizon.state[i][0];
		d(1) = obstacle_horizon.state[i][1] - usv_horizon.state[i][1];
		dist = d.norm();

		R = 0;
		C = 0;
		mu = 0;

		if (dist < D_CLOSE_){

			v_o(0) = obstacle_horizon.state[i][3];
			v_o(1) = obstacle_horizon.state[i][4];
			rot2d(obstacle_horizon.state[i][2],v_o);

			v_s(0) = usv_horizon.state[i][3];
			v_s(1) = usv_horizon.state[i][3];
			rot2d(usv_horizon.state[i][2],v_s);

			psi_o = obstacle_horizon.state[i][2];
			while(psi_o <= -M_PI) phi += 2*M_PI;
			while (psi_o > M_PI) phi -= 2*M_PI;

			phi = atan2(d(1),d(0)) - usv_horizon.state[i][2];
			while(phi <= -M_PI) phi += 2*M_PI;
			while (phi > M_PI) phi -= 2*M_PI;

			psi_rel = psi_o - usv_horizon.state[i][2];
			while(psi_rel < -M_PI) psi_rel += 2*M_PI;
			while(psi_rel > M_PI) psi_rel -= 2*M_PI;

			los = d/dist;
			los_inv = -d/dist;

			// Calculating d_safe
			if (phi < PHI_AH_*DEG2RAD){						 	// obst ahead
				d_safe_i = d_safe + usv_.getL()/2;
			}else if (phi > PHI_OT_*DEG2RAD){					// obst behind
				d_safe_i = d_safe + usv_.getL()/2;
			}else{
				d_safe_i = d_safe + usv_.getW()/2;
			}

			phi_o = atan2(-d(1),-d(0)) - obstacle_horizon.state[i][2];
			while(phi_o <= -M_PI) phi_o += 2*M_PI;
			while (phi_o > M_PI) phi_o -= 2*M_PI;

			if (phi_o < PHI_AH_*DEG2RAD){						// ship ahead
				d_safe_i += d_safe + obstacle_vessels_[k]->model_.getL()/2;
			}else if(phi_o > PHI_OT_*DEG2RAD){			 		// ship behind
				d_safe_i += 0.5*d_safe + obstacle_vessels_[k]->model_.getL()/2;
			}else{
				d_safe_i += d_safe + obstacle_vessels_[k]->model_.getW()/2;
			}

			if (v_s.dot(v_o) > cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm() // overtaing situation
								&& v_s.norm() > v_o.norm()){
				d_safe_i = d_safe + usv_.getL()/2 + obstacle_vessels_[k]->model_.getL()/2;
			}


			if (dist < d_safe_i){
				R = (1/pow(fabs(t-t0),P_))*pow(d_safe/dist,Q_);
				k_coll = K_COLL_*obstacle_vessels_[k]->model_.getL()*obstacle_vessels_[k]->model_.getL();
				C = k_coll*pow((v_s-v_o).norm(),2);
			}



			// Overtaken by obstacle
			OT = v_s.dot(v_o) > cos(PHI_OT_*DEG2RAD)*v_s.norm()*v_o.norm()
					&& v_s.norm() < v_o.norm();
			// Obstacle on starboard side
			SB = phi < 0;
			// Obstacle Head-on
			HO = v_o.norm() > 0.05
					&& v_s.dot(v_o) < -cos(PHI_HO_*DEG2RAD)*v_s.norm()*v_o.norm()
					&& v_s.dot(los) > cos(PHI_AH_*DEG2RAD)*v_s.norm();
			// Crossing situation
			CR = (v_s.dot(v_o) < cos(PHI_CR_*DEG2RAD)*v_s.norm()*v_o.norm() // changed
					&& (SB && psi_rel > 0 ));

			mu = ( SB && HO ) || ( CR && !OT);

		}

		H0 = C*R + KAPPA_*mu;

		if (H0 > H1){
			H1 = H0;  // Maximizing the cost with regards to time
		}
	}

	H2 = K_P_*(1-P_ca) + K_CHI_*pow(Chi_ca,2) + Delta_P(P_ca) + Delta_Chi(Chi_ca);
	cost =  H1 + H2 + H3;

	// Print H1 and H2 for P==X
    //	ROS_DEBUG_COND_NAMED(P_ca == 0.5,"Testing","Chi: %0.0f   \tP: %0.1f  \tH1: %0.2f  \tH2: %0.2f  \tcost: %0.2f", Chi_ca*RAD2DEG, P_ca, H1, H2, cost);
    //	ROS_DEBUG_COND_NAMED(k == 2 , "Testing","Chi: %0.0f   \tP: %0.1f  \tH1: %0.2f  \tH2: %0.2f  \tcost: %0.2f", Chi_ca*RAD2DEG, P_ca, H1, H2, cost);
    //	ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","Chi: " << Chi_ca*RAD2DEG << "  \tSB " << sb << "\tCR " << cr << "\tHO " << ho << "\tOT " << ot);
    // Print H1 and H2 for all P
    //	ROS_DEBUG_NAMED("Testing","Chi: %0.0f   \tP: %0.1f  \tH1: %0.2f  \tH2: %0.2f  \tcost: %0.2f", Chi_ca*RAD2DEG, P_ca, H1, H2, cost);
    // Print mu_1 and mu_2
    //	ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","Chi: " << Chi_ca*RAD2DEG << "  \tSB " << mu_1  << " CR " << mu_2 << " OT " << mu_3);
    //	ROS_DEBUG_STREAM_COND_NAMED(P_ca == 1, "Testing","psi_o: "<<psi_o*RAD2DEG<<"\tpsi_s: "<<psi_s*RAD2DEG);

	return cost;
}

/**
 * @brief Calculation used for cost function to penalize speed correction changes.
 * 
 * @remark From: https://github.com/olesot/ros_asv_system
 * 
 * @param P_ca Speed multiplier correction candidate
 * @return double Cost of action candidate
 */
double SimulationBasedMPC::Delta_P(double P_ca){

	return K_DP_*std::abs(P_ca_last_ - P_ca);		// 0.5
}

/**
 * @brief Calculation used for cost function to penalize yaw correction changes.
 * 
 * @remark From: https://github.com/olesot/ros_asv_system
 * 
 * @param Chi_ca Yaw correction candidate
 * @return double Cost of action candidate
 */
double SimulationBasedMPC::Delta_Chi(double Chi_ca){
	double dChi = Chi_ca - Chi_ca_last_;
	if (dChi > 0){
		return K_DCHI_P_*pow(dChi,2); 		// 0.006 0.45
	}else if (dChi < 0){
		return K_DCHI_SB_*pow(dChi,2);				// 0.35
	}else{
		return 0;
	}
}

// Utils
void rot2d(double yaw, Eigen::Vector2d &res){
	Eigen::Matrix2d R;
	R << cos(yaw), -sin(yaw),
		 sin(yaw), cos(yaw);
	res = R*res;
}

/**
 * @brief Visualize a SBMPC path
 * 
 * @param path The path parametrized as a OGRLineString
 */
void SimulationBasedMPC::visualizePath(OGRLineString& path){
    path_viz_.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point point1;
    geometry_msgs::Point prev_point;
    for (int i = 0; i<path.getNumPoints();i++){

        Eigen::Vector3d global_point;
        global_point(0) = path.getX(i);
        global_point(1) = path.getY(i);
        global_point(2) = 0;
        Eigen::Vector3d local_point;
        geo_converter_.convertSynced("WGS84",global_point,"global_enu",&local_point);

        point1.x = local_point(0);
        point1.y = local_point(1);
        if(i==0){
            prev_point.x = point1.x;
            prev_point.y = point1.y;
        }
        path_viz_.points.push_back(prev_point);
        path_viz_.points.push_back(point1);

        prev_point.x = point1.x;
        prev_point.y = point1.y;
       
    }
    path_viz_pub_.publish(path_viz_);
}

/**
 * @brief Clear visual path from rviz and clear storing vector.
 * 
 */
void SimulationBasedMPC::clearVisualPath(){
    path_viz_.points.clear();
    path_viz_.action = visualization_msgs::Marker::DELETEALL;
    path_viz_pub_.publish(path_viz_);
}