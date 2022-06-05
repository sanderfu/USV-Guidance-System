#include "usv_colav/colav.h"

Colav::Colav(const ros::NodeHandle& nh) : 
nh_(nh),
geo_converter_("COLAV",false,true,nh)
{   
    bool parameter_load_error = false;
    std::string map_name;
    if(!ros::param::get("map_name",map_name)) parameter_load_error = true;
    if(!ros::param::get("id",ownship_id_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/P_",P_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/Q_",Q_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/D_CLOSE_",D_CLOSE_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/D_SAFE_",D_SAFE_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_COLL_",K_COLL_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/PHI_AH_",PHI_AH_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/PHI_OT_",PHI_OT_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/PHI_HO_",PHI_HO_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/PHI_CR_",PHI_CR_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/KAPPA_",KAPPA_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_P_",K_P_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_CHI_",K_CHI_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_DP_",K_DP_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_DCHI_SB_",K_DCHI_SB_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_DCHI_P_",K_DCHI_P_)) parameter_load_error = true;
    if(!ros::param::get("colav/cost_function/K_CORR_",K_CORR_)) parameter_load_error = true;
    if(!ros::param::get("colav/offsets/Chi_ca_",Chi_ca_)) parameter_load_error = true;
    if(!ros::param::get("colav/offsets/P_ca_",P_ca_)) parameter_load_error = true;
    if(!ros::param::get("colav/update_frequency",update_frequency_)) parameter_load_error = true;
    if(!ros::param::get("colav/prediction_time",prediction_time_)) parameter_load_error = true;
    if(!ros::param::get("colav/verbose",verbose_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }

    // Convert degrees to rad
    for(int i=0; i<Chi_ca_.size(); i++){
        Chi_ca_[i]*=DEG2RAD;
    }

    map_service_ = new MapService(map_name);
    //Warning: I do not know if this will onlky surpress warnings for GDAL related to this module or also the mission planner. 
    ROS_WARN_STREAM("GDAL errors are supressed!");
    CPLPushErrorHandler(CPLQuietErrorHandler);
    
    ROS_INFO_STREAM_COND(verbose_,"COLAV System started, waiting for first odometry and LOS setpoint from USV");
    latest_odom_ = *ros::topic::waitForMessage<nav_msgs::Odometry>("odom",nh_);
    latest_los_setpoint_ = *ros::topic::waitForMessage<geometry_msgs::Twist>("los/setpoint",nh_);
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh_);
    ROS_INFO_STREAM_COND(verbose_,"Odometry and LOS setpoint received");

    correction_pub_ = nh_.advertise<geometry_msgs::Twist>("colav/correction",1,false);
    colav_data_pub_ = nh_.advertise<usv_msgs::Colav>("colav/debug",1,false);

    odom_sub_ = nh_.subscribe("odom",10,&Colav::odomCb,this);
    los_setpoint_sub_ = nh_.subscribe("los/setpoint",10,&Colav::losSetpointSub,this);
    obstacle_sub_ = nh_.subscribe("/obstacles",10,&Colav::obstacleCb,this);
    system_reinit_sub_ = nh_.subscribe("mc/system_reinit",10,&Colav::reinitCb,this);

    //Set course action pairs
    bool multiple_changes = false;
    for(auto chi_first_it=Chi_ca_.begin(); chi_first_it!=Chi_ca_.end();chi_first_it++){
        if(multiple_changes){
            for(auto chi_second_it=Chi_ca_.begin(); chi_second_it!=Chi_ca_.end(); chi_second_it++){
                if((*chi_first_it<0)!=(*chi_second_it<0) && *chi_first_it!=0) continue;
                Chi_ca_sequences_.push_back({*chi_first_it,*chi_second_it});
            }
        
        } else{
            Chi_ca_sequences_.push_back({*chi_first_it});
        }
    }

    Chi_ca_last_ = 0.0;
    P_ca_last_ = 1.0;

    colav_msg_.obstacle_pose_local = std::vector<geometry_msgs::Point>(4);

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

    main_loop_timer_ = nh_.createTimer(ros::Duration(1/update_frequency_),&Colav::mainLoop,this);

}
/**
 * @brief Callback when receiving odometry from vessel on which the Colav system is running.
 * 
 * @param odom Odometry message
 */
void Colav::odomCb(const nav_msgs::Odometry& odom){
    latest_odom_ = odom;
}

/**
 * @brief Callback when receiving LOS setpoints from vessel on which the Colav system is running.
 * 
 * @param msg Twist message (linear.x=speed setpoint, angular.z=yaw setpoint)
 */
void Colav::losSetpointSub(const geometry_msgs::Twist& msg){
    latest_los_setpoint_ = msg;
}

/**
 * @brief Callback when receiving obstacle from obstacle tracking system on vessel on which the Colav system is running
 * 
 * @param msg Obstacle message (custom message type)
 */
void Colav::obstacleCb(const usv_simulator::obstacle& msg){
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
        obstacle_vessels_[msg.id]->latest_observation_ = ros::Time::now();
        obstacle_vessels_[msg.id]->id = msg.id;
    }else{
        obstacle_vessels_[msg.id]->latest_obstacle_state_ = state;
        obstacle_vessels_[msg.id]->latest_observation_ = ros::Time::now();
    }
    colav_msg_.obstacle_pose_local[obstacle_vessels_[msg.id]->id].x = state[0];
    colav_msg_.obstacle_pose_local[obstacle_vessels_[msg.id]->id].y = state[1];
    colav_msg_.obstacle_pose_local[obstacle_vessels_[msg.id]->id].z = state[2];
    
}

/**
 * @brief Calculate the lowest-cost control offsets for speed and yaw.
 * 
 * @param u_corr_best Best speed correction (multiplier) (-1,0,0.5,1)
 * @param psi_corr_best Best yaw correction  
 */
void Colav::getBestControlOffset(double& u_corr_best, double& psi_corr_best){
    ros::Time start = ros::Time::now();
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
    colav_msg_.obstacles_odom.clear();
    colav_msg_.obstacle_desync.clear();
    colav_msg_.evaluateCandidate_time.clear();
    colav_msg_.checkForCollision_time.clear();
    colav_msg_.simulateOwnship_time.clear();
    colav_msg_.costFnc_time.clear();

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
    
    colav_msg_.ownship_pose_local.x = state[0];
    colav_msg_.ownship_pose_local.y = state[1];
    colav_msg_.ownship_pose_local.z = state[2];

    //Push back odom to colav message
    colav_msg_.ownship_odom = latest_odom_;

    //Evaluate desync between ownship and obstacles
    for (auto it = obstacle_vessels_.begin(); it!=obstacle_vessels_.end();it++){
        colav_msg_.obstacle_desync.push_back(ros::Duration(ros::Time::now()-(*it).second->latest_observation_).toSec());
    }


    control_candidate_map.clear();
    for(auto p_it=P_ca_.begin(); p_it!=P_ca_.end();p_it++){
        for(auto chi_seq_it = Chi_ca_sequences_.begin(); chi_seq_it!=Chi_ca_sequences_.end();chi_seq_it++){
            ros::Time evaluateCandidate_start = ros::Time::now();
            double cost_i = 0.0;
            candidate_violating_colreg_14_ = false;
            candidate_violating_colreg_15_ = false;
            //Simulate vessel
            state_type state_copy = state;

            double sim_time = prediction_time_/(*chi_seq_it).size();
            ModelLibrary::simulatedHorizon full_horizon_;
            double Chi_ca_last = Chi_ca_last_;
            for(auto chi_it = (*chi_seq_it).begin(); chi_it != (*chi_seq_it).end(); chi_it++){
                double cost_tmp = 0.0;
                double t_offset = sim_time * (chi_it-(*chi_seq_it).begin());
                ros::Time simulateOwnship_start = ros::Time::now();
                ModelLibrary::simulatedHorizon horizon = usv_.simulateHorizonAdaptive(state_copy,(latest_los_setpoint_.linear.x)*(*p_it),(latest_los_setpoint_.angular.z+(*chi_it)),sim_time);
                colav_msg_.simulateOwnship_time.push_back(ros::Duration(ros::Time::now()-simulateOwnship_start).toSec());
                //Avoid collision in COLREG compliant manner (if no vessels, penalize only the course and speed correction)
                if(obstacle_vessels_.size()!=0){
                    for (auto it = obstacle_vessels_.begin(); it!=obstacle_vessels_.end();it++){
                        if((*it).second->id == ownship_id_) continue;
                        double key = it->first;
                        ros::Time costFnc_start = ros::Time::now();
                        cost_k = costFnc(horizon, *(*it).second, *p_it, *chi_it, key, t_offset,P_ca_last_,Chi_ca_last);
                        colav_msg_.costFnc_time.push_back(ros::Duration(ros::Time::now()-costFnc_start).toSec());
                        if (cost_k > cost_tmp){
                            cost_tmp = cost_k;	// Maximizing cost associated with this scenario
                        }
                        //Add obstacle odometries to colav message if first option
                        if(chi_seq_it == Chi_ca_sequences_.begin() && p_it==P_ca_.begin()){
                            nav_msgs::Odometry obstacle_odom;
                            obstacle_odom.pose.pose.position.x = (*it).second->latest_obstacle_state_[0];
                            obstacle_odom.pose.pose.position.y = (*it).second->latest_obstacle_state_[1];
                            tf::Quaternion q;
                            q.setRPY(0,0,(*it).second->latest_obstacle_state_[2]);
                            tf::quaternionTFToMsg(q,obstacle_odom.pose.pose.orientation);
                            obstacle_odom.twist.twist.linear.x = (*it).second->latest_obstacle_state_[3];
                            obstacle_odom.twist.twist.linear.y = (*it).second->latest_obstacle_state_[4];
                            obstacle_odom.twist.twist.angular.z = (*it).second->latest_obstacle_state_[5];
                            colav_msg_.obstacles_odom.push_back(obstacle_odom);
                        }
                    }
                } else{
                    cost_i = K_P_*(1-*p_it) + K_CHI_*pow(*chi_it,2) + Delta_P(*p_it,P_ca_last_) + Delta_Chi(*chi_it,Chi_ca_last,false);
                }

                if(candidate_violating_colreg_14_) ROS_WARN_STREAM("Candidate violating colreg 14 with correction: " << *chi_it * RAD2DEG); 
                if(candidate_violating_colreg_15_) ROS_WARN_STREAM("Candidate violating colreg 15 with correction: " << *chi_it * RAD2DEG); 

                full_horizon_.state.insert(full_horizon_.state.end(), horizon.state.begin(), horizon.state.end());
                full_horizon_.time.insert(full_horizon_.time.end(), horizon.time.begin(), horizon.time.end());
                cost_i+=cost_tmp;
                Chi_ca_last = *chi_it;
            }

            //Simulation horizon to LineString and add to data message
            
            usv_msgs::ColavPath msg_path;
            usv_msgs::ColavPathElement path_element;
            for(auto hor_it = full_horizon_.state.begin(); hor_it!=full_horizon_.state.end();hor_it++){
                point_local(0) = hor_it->at(0);
                point_local(1) = hor_it->at(1);
                point_local(2) = 0;
                geo_converter_.convertSynced("global_enu",point_local,"WGS84",&point_global);
                path_element.lon = point_global.x();
                path_element.lat = point_global.y();
                path_element.course = hor_it->at(2);
                msg_path.path.push_back(path_element);
            }
            colav_msg_.path_options.push_back(msg_path);
            colav_msg_.cost_options.push_back(cost_i);
            

            //Add candidate to candidate list
            control_candidate_map.insert(std::make_pair(cost_i,controlCandidate(*p_it,(*chi_seq_it)[0],full_horizon_,cost_i,candidate_violating_colreg_14_)));

            colav_msg_.evaluateCandidate_time.push_back(ros::Duration(ros::Time::now()-evaluateCandidate_start).toSec());
        }
    }

    //Find best candidate from candidate map (lowest cost without collision)
    ros::Time checkForCollision_start = ros::Time::now();
    for(auto it = control_candidate_map.begin(); it!=control_candidate_map.end(); it++){
        OGRLineString path;
        usv_msgs::ColavPath msg_path;
        usv_msgs::ColavPathElement path_element;
        for(auto hor_it = (*it).second.horizon_.state.begin(); hor_it!=(*it).second.horizon_.state.end();hor_it++){
            if ((*it).second.horizon_.time[hor_it-(*it).second.horizon_.state.begin()]>5){
                break;
            }
            point_local(0) = hor_it->at(0);
            point_local(1) = hor_it->at(1);
            point_local(2) = 0;
            geo_converter_.convertSynced("global_enu",point_local,"WGS84",&point_global);
            path.addPoint(point_global(0),point_global(1));
        }
        if(!map_service_->intersects(&path,LayerID::COLLISION)){
            cost = (*it).second.cost_;
            u_corr_best = (*it).second.p_cand_;
            psi_corr_best = (*it).second.chi_cand_; 
            choosen_path = path;

            ROS_WARN_STREAM_COND((*it).second.colreg_violation_,"Choosen candidate violates colregs");

            //Update colav message
            colav_msg_.path = msg_path;
            colav_msg_.cost = cost;
            colav_msg_.speed_correction = u_corr_best;
            colav_msg_.course_correction = psi_corr_best;
            colav_msg_.time = ros::Time::now();
            break;
        }
    }
    colav_msg_.checkForCollision_time.push_back(ros::Duration(ros::Time::now()-checkForCollision_start).toSec());


    P_ca_last_ = u_corr_best;
	Chi_ca_last_ = psi_corr_best;
    ROS_INFO_STREAM_COND(verbose_,"Path num points: " << choosen_path.getNumPoints());
    ROS_INFO_STREAM_COND(verbose_,"Best cost: " << cost);
    colav_msg_.getBestControlOffset_time=ros::Duration(ros::Time::now()-start).toSec();
    ros::Time stop = ros::Time::now();
}

/**
 * @brief The main loop of the Colav System. 
 * 
 * @details Called periodically by designated timer. Calculates best offset from latest available information and pulishes this offset.
 * 
 * @param e TimerEveent parameter, currently not used.
 */
void Colav::mainLoop(const ros::TimerEvent& e){
    double u_os, psi_os;
    getBestControlOffset(u_os,psi_os);
    

    //Publish COLAV correction
    geometry_msgs::Twist offset;
    offset.linear.x = u_os;
    offset.angular.z = psi_os;
    correction_pub_.publish(offset);

    //Publish debug/post-visualization message
    colav_data_pub_.publish(colav_msg_);
    ROS_INFO_STREAM_COND(verbose_, "Offset u_os: " << u_os << " Offset psi_os: " << psi_os*RAD2DEG);
}

/**
 * @brief Reinitialize COLAV system upon instruction from a Monte Carlo Supervisor or other future system.
 * 
 * @param msg Renitialize message
 */
void Colav::reinitCb(const usv_msgs::reinit& msg){
    main_loop_timer_.stop();
    latest_odom_ = *ros::topic::waitForMessage<nav_msgs::Odometry>("odom",nh_);
    latest_los_setpoint_ = *ros::topic::waitForMessage<geometry_msgs::Twist>("los/setpoint",nh_);
    Chi_ca_last_ = 0.0;
    P_ca_last_ = 1.0;
    main_loop_timer_.start();
}

/**
 * @brief Cost function to avoid other vessels in COLREG compliant manner. Calculate cost for a given obstacle vessel horizon.
 * 
 * @remark This cost function is based on the work in https://github.com/ingerbha/ros_asv_system
 * 
 * @param usv_horizon The simulated horizon for the ownship
 * @param obstacle_horizon The simulated horizon for an obstacle ship
 * @param P_ca Speed correction candidate giving simulated ownship horizon (multiplier)
 * @param Chi_ca Yaw correction giving simulated ownship horizon
 * @param k ID of obstacle ship
 * @return double Cost of action candidate.
 */
double Colav::costFnc(ModelLibrary::simulatedHorizon& usv_horizon, obstacleVessel& obstacle_vessel, double P_ca, double Chi_ca, int k, double t_offset, double P_ca_last, double Chi_ca_last)
{
	double dist, phi, psi_o, phi_o, psi_rel, R, C, k_coll, d_safe_i;
	Eigen::Vector2d d, los, los_inv, v_o, v_s, delta_p_obs;
	bool mu, OT, SB, HO, CR;
	double combined_radius = usv_.getL() + obstacle_vessels_[k]->model_.getL();
	double d_safe = D_SAFE_;
	double H0 = 0;
	double H1 = 0;
	double H2 = 0;
    double H3 = 0;
	double cost = 0;
	double t0 = t_offset;
    double t = 0;
    double t_prev = 0;
    state_type obstacle_state = obstacle_vessel.latest_obstacle_state_;

    candidate_violating_colreg_14_ = false;
    candidate_violating_colreg_15_ = false;

    std::set<double> dist_set;

	for (int i = 0; i < usv_horizon.steps; i++){

        t = usv_horizon.time[i];
        delta_p_obs(0)=(t-t_prev)*obstacle_state[3];
        delta_p_obs(1)=(t-t_prev)*obstacle_state[4];
        rot2d(obstacle_state[2],delta_p_obs);
        obstacle_state[0] += delta_p_obs.x();
        obstacle_state[1] += delta_p_obs.y();
        t_prev = t;

		d(0) = obstacle_state[0] - usv_horizon.state[i][0];
		d(1) = obstacle_state[1] - usv_horizon.state[i][1];
		dist = d.norm();
        dist_set.insert(dist);


		R = 0;
		C = 0;
		mu = 0;

		if (dist < D_CLOSE_){
			v_o(0) = obstacle_state[3];
			v_o(1) = obstacle_state[4];
			rot2d(obstacle_state[2],v_o);

			v_s(0) = usv_horizon.state[i][3];
			v_s(1) = usv_horizon.state[i][4];
			rot2d(usv_horizon.state[usv_horizon.state.size()-1][2],v_s);
        

			psi_o = obstacle_state[2];
			while(psi_o <= -M_PI) phi += 2*M_PI;
			while (psi_o > M_PI) phi -= 2*M_PI;

			phi = atan2(d(1),d(0)) - usv_horizon.state[usv_horizon.state.size()-1][2];
			while(phi <= -M_PI) phi += 2*M_PI;
			while (phi > M_PI) phi -= 2*M_PI;

			psi_rel = psi_o - usv_horizon.state[usv_horizon.state.size()-1][2];
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

			phi_o = atan2(-d(1),-d(0)) - obstacle_state[2];
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

            
            /*if(Chi_ca==0){
                std::cout << "Dist: " << dist << "d_safe: " << d_safe << " d_safe_i:" << d_safe_i <<std::endl;
            }*/


			if (dist < d_safe && t!=t0){
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
			CR = (v_s.dot(v_o) < cos(PHI_CR_*DEG2RAD)*v_s.norm()*v_o.norm()) && (SB && psi_rel > 0 );

			mu = ( SB && HO ) || ( SB && CR && !OT);
            if(mu){
                candidate_violating_colreg_14_=( SB && HO );
                candidate_violating_colreg_15_=( SB && CR && !OT);
            }

		}

		H0 = C*R + KAPPA_*mu;

		if (H0 > H1){
			H1 = H0;  // Maximizing the cost with regards to time
		}
        
	}
	H2 = K_P_*(1-P_ca) + K_CHI_*(pow(Chi_ca,2)) + Delta_P(P_ca,P_ca_last) + Delta_Chi(Chi_ca, Chi_ca_last,mu) + K_CORR_*(Chi_ca!=0);
	cost =  H1 + H2 + H3;

    ROS_WARN_STREAM_COND(Chi_ca==0,"Cost of zero option" << cost);
    ROS_WARN_STREAM_COND(Chi_ca==0,"H1:" << H1 << " H2: " << H2 << " H3: " << H3); 
    ROS_WARN_STREAM_COND(Chi_ca==0,"Smallest dist: " << *(dist_set.begin()) << " deltaP: " << Delta_P(P_ca,P_ca_last) << " deltaChi: " <<  Delta_Chi(Chi_ca, Chi_ca_last,mu)); 

	return cost;
}

/**
 * @brief Calculation used for cost function to penalize speed correction changes.
 * 
 * @param P_ca Speed multiplier correction candidate
 * @return double Cost of action candidate
 */
double Colav::Delta_P(double P_ca, double P_ca_last){

	return K_DP_*std::abs(P_ca_last - P_ca);		// 0.5
}

/**
 * @brief Calculation used for cost function to penalize yaw correction changes.
 * 
 * @param Chi_ca Yaw correction candidate
 * @return double Cost of action candidate
 */
double Colav::Delta_Chi(double Chi_ca, double Chi_ca_last, bool mu){
	double dChi = Chi_ca - Chi_ca_last;
	if (dChi > 0){
		return K_DCHI_P_*pow(dChi,2); 		// 0.006 0.45
	}else if (dChi < 0){
		return K_DCHI_SB_*pow(dChi,2);				// 0.35
	}else{
		return 0;
	}
}

/**
 * @brief Rotation matrix for 2d vectors
 * 
 * @param yaw 
 * @param res 
 */
void rot2d(double yaw, Eigen::Vector2d &res){
	Eigen::Matrix2d R;
	R << cos(yaw), -sin(yaw),
		 sin(yaw), cos(yaw);
	res = R*res;
}

/**
 * @brief Visualize a COlav system path
 * 
 * @param path The path parametrized as a OGRLineString
 */
void Colav::visualizePath(OGRLineString& path){
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
void Colav::clearVisualPath(){
    path_viz_.points.clear();
    path_viz_.action = visualization_msgs::Marker::DELETEALL;
    path_viz_pub_.publish(path_viz_);
}