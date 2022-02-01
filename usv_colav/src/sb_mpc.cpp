#include "usv_colav/sb_mpc.h"

SimulationBasedMPC::SimulationBasedMPC(const ros::NodeHandle& nh) : 
nh_(nh),
map_client_(&nh_),
geodetic_client_(&nh_),
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
K_DCHI_P_(1.2)			//   1.2
{
    ROS_INFO_STREAM("SB_MPC started, waiting for first odometry and LOS setpoint from USV");
    latest_odom_ = *ros::topic::waitForMessage<nav_msgs::Odometry>("/Viknes830/odom",nh_);
    latest_los_setpoint_ = *ros::topic::waitForMessage<geometry_msgs::Twist>("/Viknes830/los/setpoint",nh_);
    ROS_INFO_STREAM("Odometry received");

    geo_converter_.addFrameByEPSG("WGS84",4326);
    geo_converter_.addFrameByENUOrigin("global_enu",40.5612,-73.9761,0);

    correction_pub_ = nh_.advertise<geometry_msgs::Twist>("colav/correction",1,false);

    odom_sub_ = nh_.subscribe("/Viknes830/odom",1,&SimulationBasedMPC::odomCb,this);
    los_setpoint_sub_ = nh_.subscribe("/Viknes830/los/setpoint",1,&SimulationBasedMPC::losSetpointSub,this);
    main_loop_timer_ = nh_.createTimer(ros::Duration(1),&SimulationBasedMPC::mainLoop,this);

    //Set course action alternatives
    double courseOffsets[] = {-90.0,-75.0,-60.0,-45.0,-30.0,-15.0,0.0,15.0,30.0,45.0,60.0,75.0,90.0};
    double sizeCO = sizeof(courseOffsets)/sizeof(courseOffsets[0]);
    for (int i = 0; i < sizeCO; i++){
    	courseOffsets[i] *= DEG2RAD;
    }
    Chi_ca_.assign(courseOffsets, courseOffsets + sizeof(courseOffsets)/sizeof(courseOffsets[0]));

    //Set speed action alternatives
    double speedOffsets[] = {-1,0,0.5,1};
    P_ca_.assign(speedOffsets, speedOffsets + sizeof(speedOffsets)/sizeof(speedOffsets[0]));

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

void SimulationBasedMPC::odomCb(const nav_msgs::Odometry& odom){
    latest_odom_ = odom;
}

void SimulationBasedMPC::losSetpointSub(const geometry_msgs::Twist& msg){
    latest_los_setpoint_ = msg;
}

void SimulationBasedMPC::getBestControlOffset(double& u_corr_best, double& psi_corr_best){
    double cost = INFINITY;
    double cost_k = 0.0;
    tfScalar roll, pitch,yaw;
    tf::Quaternion q;
    Eigen::Vector3d point_local;
    Eigen::Vector3d point_global;
    OGRLineString choosen_path;

    for(auto chi_it = Chi_ca_.begin(); chi_it!=Chi_ca_.end();chi_it++){
        for(auto p_it=P_ca_.begin(); p_it!=P_ca_.end();p_it++){
            double cost_i = 0.0;
            //Simulate vessel
            state_type state(6);
            state[0] = latest_odom_.pose.pose.position.x; 
            state[1] = latest_odom_.pose.pose.position.y;
            tf::quaternionMsgToTF(latest_odom_.pose.pose.orientation,q);
            tf::Matrix3x3 mat(q);
            mat.getRPY(roll,pitch,yaw);
            state[2] = yaw;
            state[3] = latest_odom_.twist.twist.linear.x;
            state[4] = latest_odom_.twist.twist.linear.y;
            state[5] = latest_odom_.twist.twist.angular.z;
            //ros::Time start = ros::Time::now();
            ModelLibrary::simulatedHorizon horizon = usv_.simulateHorizonAdaptive(state,(latest_los_setpoint_.linear.x+1)*(*p_it),latest_los_setpoint_.angular.z+(*chi_it),60);
            //ros::Time end = ros::Time::now();
            //ROS_INFO_STREAM("Simulation took: " << (end-start).toSec() << " [s]");


            //Simulation horizon to LineString
            OGRLineString path;
            std::string global_enu = "global_enu";
            std::string global_wgs = "WGS84";

            for(auto hor_it = horizon.state.begin(); hor_it!=horizon.state.end();hor_it++){
                point_local(0) = hor_it->at(0);
                point_local(1) = hor_it->at(1);
                point_local(2) = 0;
                geo_converter_.convert(global_enu,point_local,global_wgs,&point_global);
                path.addPoint(point_global(0),point_global(1));
            }


            //Check path for collision
            if (map_client_.collision(&path)){
                cost_i += 100;
            }

            if((*p_it)!=1 || (*chi_it)!=0){
                cost_i += 5;
            }
            
            //Choose action based on minimized cost
            if (cost_i<cost){
                cost = cost_i;
                u_corr_best = *p_it;
                psi_corr_best = *chi_it; 
                choosen_path = path;
            }

        }
    }
    visualizePath(choosen_path);
    ROS_INFO_STREAM("Path num points: " << choosen_path.getNumPoints());
    ROS_INFO_STREAM("Best cost: " << cost);
    P_ca_last_ = u_corr_best;
	Chi_ca_last_ = psi_corr_best;
}

 void SimulationBasedMPC::mainLoop(const ros::TimerEvent& e){
    double u_os, psi_os;
    ros::Time start = ros::Time::now();
    getBestControlOffset(u_os,psi_os);
    ros::Time stop = ros::Time::now();
    ROS_INFO_STREAM("Offset u_os: " << u_os << " Offset psi_os: " << psi_os*RAD2DEG);
    ROS_INFO_STREAM("Getting offset took: " << (stop-start).toSec() << " [s]");
}

double SimulationBasedMPC::Delta_P(double P_ca){

	return K_DP_*std::abs(P_ca_last_ - P_ca);		// 0.5
}

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

void SimulationBasedMPC::visualizePath(OGRLineString& path){
    geometry_msgs::Point point1;
    geometry_msgs::Point prev_point;
    for (int i = 0; i<path.getNumPoints();i++){

        Eigen::Vector3d global_point;
        global_point(0) = path.getX(i);
        global_point(1) = path.getY(i);
        global_point(2) = 0;
        Eigen::Vector3d local_point;
        geo_converter_.convert("WGS84",global_point,"global_enu",&local_point);

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