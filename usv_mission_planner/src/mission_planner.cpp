#include "usv_mission_planner/mission_planner.h"

/**
 * @brief Construct a new Mission Planner object
 * 
 * @details Register publishers and subscribers, load relevant parameters, load and publish gpx path(optionally) 
 * preprocess mission region if not already done or forced to (optionally) and construct objects needed for path search functionality.
 * 
 * @param nh ROS NodeHandle
 */
MissionPlanner::MissionPlanner(const ros::NodeHandle& nh): nh_(nh){
    GDALAllRegister();
    path_pub_ = nh_.advertise<geometry_msgs::Pose>("mission_planner/geo_waypoint",1,false);
    speed_pub_ = nh_.advertise<geometry_msgs::Twist>("mission_planner/desired_speed",1,true);
    region_available_pub_ = nh_.advertise<std_msgs::Bool>("mission_planner/region_available",1,true);
    mission_done_pub_ = nh_.advertise<std_msgs::Bool>("mission_planner/done",1,false);
    waypoint_reached_sub_ = nh_.subscribe("los/waypoint_reached",1,&MissionPlanner::waypointReachedCb,this);
    search_service_ = nh_.advertiseService("SearchGlobalPath",&MissionPlanner::search,this);

    bool parameter_load_error = false;
    if(!ros::param::get("mission_planner/gpx_name",gpx_name_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/preprocessed_map",preprocessed_map_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/map_name",map_name_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/map_extent",mission_region_extent_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/desired_speed",desired_speed_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }

    if(!preprocessed_map_){
        std::cout << "Start preprocessing map" << std::endl;
        MapPreprocessor preprocessor;
        extractorRegion r(mission_region_extent_[0],mission_region_extent_[1],mission_region_extent_[2],mission_region_extent_[3]);
        preprocessor.run(map_name_,r);
         std::cout << "Done preprocessing map" << std::endl;
    }
    region_available_pub_.publish(std_msgs::Bool());

    map_service_ = new MapService(map_name_);
    std::pair<OGRPoint,OGRPoint> map_extent = map_service_->getMapExtent();
    tree_ = new Quadtree(map_extent.first,map_extent.second,map_service_->getDataset(),map_name_,map_service_,false);
    vessel_model_ = new ModelLibrary::Viknes830();
    search_alg_ = new HybridAStar(tree_,vessel_model_,map_service_,mission_name_);
    ROS_INFO_STREAM("Done initializing MissionPlanner");

}

/**
 * @brief Publish a path stored in the path_ container.
 * 
 * @todo Change from Pose to PoseArray. This has consequences for LOS too.
 * 
 */
void MissionPlanner::publishPath(){
    geometry_msgs::Pose wpt_msg;
    for(auto path_it = path_.begin(); path_it!=path_.end();path_it++){
        wpt_msg.position.x = (*path_it)->pose->x();
        wpt_msg.position.y = (*path_it)->pose->y();
        path_pub_.publish(wpt_msg);
        ros::Duration(0.01).sleep();
    }
}

/**
 * @brief Publish a desired speed stored in the desired_speed_ variable.
 * 
 */
void MissionPlanner::publishSpeed(){
    geometry_msgs::Twist msg;
    msg.linear.x = desired_speed_;
    speed_pub_.publish(msg);
}

/**
 * @brief Service function to find an optimized path.
 * 
 * @details The search is either from latest vessel omodmetry or from specified position based on service request.
 * 
 * @param req Service request
 * @param res Service response
 * @return true The service call function completed without failure
 * @return false The service call function failed
 */
bool MissionPlanner::search(usv_mission_planner::search::Request &req, usv_mission_planner::search::Response &res){
    ROS_INFO_STREAM("Start search in mission planner");
    //Crate mission directory, add timestamp before name
    mission_path_ = ros::package::getPath("usv_mission_planner")+"/data/missions/"+req.mission_name.data+"/";
    if(!boost::filesystem::exists(mission_path_)){
        boost::filesystem::create_directories(mission_path_);
    }

    if(req.predefined){
        std::string gpx_path = mission_path_+gpx_name_+".gpx";
        GDALDataset* gpx_ds = (GDALDataset*) GDALOpenEx(gpx_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
        if(gpx_ds == NULL){
            ROS_ERROR_STREAM("Failed to load gpx path file: " << gpx_path);
            ros::shutdown();
        }
        OGRLineString* route = gpx_ds->GetLayerByName("routes")->GetFeature(0)->GetGeometryRef()->toLineString();
        for(int i=0; i<route->getNumPoints(); i++){
            state_type state = {route->getX(i),route->getY(i),0,0,0,0};
            path_.push_back(new extendedVertex(i,state));
        }
        ros::Duration(1.0).sleep();
        publishPath();
        publishSpeed();
        return true;
    }

    search_alg_->setMissionName(req.mission_name.data);

    if (req.use_odom){
        latest_gps_ = *ros::topic::waitForMessage<geometry_msgs::PoseStamped>("pose",nh_);
        std::cout << "Latest odom: " << latest_gps_.pose.position.x << latest_gps_.pose.position.y << std::endl;
        tf::Quaternion q;
        tf::quaternionMsgToTF(latest_gps_.pose.orientation,q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        search_alg_->setStart(latest_gps_.pose.position.x,latest_gps_.pose.position.y,yaw);
    } else{
        search_alg_->setStart(req.custom_start_lon,req.custom_start_lat,req.custom_start_heading);
    }
    search_alg_->setGoal(req.goal_lon,req.goal_lat,0);
    search_alg_->search();
    path_ = search_alg_->getPath();

    if(req.publish_path){
        publishPath();
        publishSpeed();
    }
    savePath();

    res.success = true;
    return true;

}

/**
 * @brief Save path stored in path_ to gpx file for debugging/visualziation in OpenCPN or custom scripts.
 * 
 */
void MissionPlanner::savePath(){
    std::string gpx_path = mission_path_+"debug"+".gpx";
    if(boost::filesystem::exists(gpx_path)){
        ROS_INFO_STREAM("GPX file already exists, deleting");
        boost::filesystem::remove(gpx_path);
    }

    GDALDriver* gpx_driver = GetGDALDriverManager()->GetDriverByName("GPX");
    GDALDataset* gpx_ds = gpx_driver->Create(gpx_path.c_str(),0,0,0,GDT_Unknown,NULL);
    OGRLayer* gpx_routes_layer = gpx_ds->CreateLayer("routes",OGRSpatialReference::GetWGS84SRS(),wkbLineString);
    OGRFeature* gpx_route_feature = OGRFeature::CreateFeature(gpx_routes_layer->GetLayerDefn());

    OGRLineString route;
    OGRPoint route_point;
    for (auto waypoint: path_){
        route_point.setX(waypoint->pose->x());
        route_point.setY(waypoint->pose->y());
        route.addPoint(&route_point);
    }
    gpx_route_feature->SetGeometry(&route);
    gpx_routes_layer->CreateFeature(gpx_route_feature);

    gpx_ds->~GDALDataset();
    ROS_INFO_STREAM("Done writing waypoints to gpx");
}

void MissionPlanner::waypointReachedCb(const geometry_msgs::Pose& msg){
    if(path_.empty()) return;
    extendedVertex* goal = *(path_.end()-1);
    if(goal->pose->x()==msg.position.x && goal->pose->y()==msg.position.y){
        //ROS_WARN_STREAM("Goal reached, informing");
        mission_done_pub_.publish(std_msgs::Bool());
    }
}

MissionPlannerClient::MissionPlannerClient(ros::NodeHandle& nh): nh_(nh){
    search_client_ = nh_.serviceClient<usv_mission_planner::search>("SearchGlobalPath");
}

/**
 * @brief Request the mission planner to search from the current vessel odometry to a specified goal.
 * 
 * @param goal_lon Goal longitude
 * @param goal_lat Goal latitude
 * @param publish_path Should path be published in ROS network?
 */
void MissionPlannerClient::searchFromOdom(double goal_lon, double goal_lat, std::string mission_name, bool publish_path){
    usv_mission_planner::search srv;
    srv.request.use_odom=true;
    srv.request.publish_path = publish_path;
    srv.request.goal_lon = goal_lon;
    srv.request.goal_lat = goal_lat;
    srv.request.mission_name.data = mission_name;
    if(!search_client_.call(srv)) ROS_ERROR_STREAM("Search service call failed!");
}

/**
 * @brief Request the mission planner to search from a custom position to a specified goal.
 * 
 * @param start_lon Start longitude
 * @param start_lat Start latitude
 * @param start_heading Start heading
 * @param goal_lon Goal longitude
 * @param goal_lat Goal latitude
 * @param publish_path Should path be published in ROS network?
 */
void MissionPlannerClient::searchFromCustom(double start_lon,double start_lat, double start_heading, double goal_lon, double goal_lat, std::string mission_name, bool publish_path){
    usv_mission_planner::search srv;
    srv.request.use_odom=false;
    srv.request.publish_path = publish_path;
    srv.request.custom_start_lon = start_lon;
    srv.request.custom_start_lat = start_lat;
    srv.request.custom_start_heading = start_heading;
    srv.request.goal_lon = goal_lon;
    srv.request.goal_lat = goal_lat;
    srv.request.mission_name.data = mission_name;
    ROS_INFO_STREAM("Calling search in MissionPlanner");
    if(!search_client_.call(srv)) ROS_ERROR_STREAM("Search service call failed!");
}

void MissionPlannerClient::loadPredefined(std::string predefined_mission_name){
    usv_mission_planner::search srv;
    srv.request.predefined = true;
    srv.request.mission_name.data = predefined_mission_name;
    ROS_INFO_STREAM("Calling search in MissionPlanner");
    if(!search_client_.call(srv)) ROS_ERROR_STREAM("Search service call failed!");
}

