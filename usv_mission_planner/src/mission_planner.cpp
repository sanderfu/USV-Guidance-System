#include "usv_mission_planner/mission_planner.h"

MissionPlanner::MissionPlanner(const ros::NodeHandle& nh): nh_(nh){
    GDALAllRegister();
    path_pub_ = nh_.advertise<geometry_msgs::Pose>("mission_planner/geo_waypoint",1,false);
    odom_sub_ = nh_.subscribe("odom",1,&MissionPlanner::odomCb,this);
    goal_sub_ = nh_.subscribe("global_goal",1,&MissionPlanner::goalCb,this);
    search_service_ = nh_.advertiseService("SearchGlobalPath",&MissionPlanner::search,this);

    bool parameter_load_error = false;
    if(!ros::param::get("mission_planner/mission_name",mission_name_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/load_from_gpx",load_from_gpx_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/gpx_name",gpx_name_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/preprocessed_map",preprocessed_map_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/map_name",map_name_)) parameter_load_error = true;
    if(!ros::param::get("mission_planner/search_immideately",search_immideately_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }

    //Crate mission directory, add timestamp before name
    mission_path_ = ros::package::getPath("usv_mission_planner")+"/data/missions/"+mission_name_+"/";
    if(!boost::filesystem::exists(mission_path_)){
        boost::filesystem::create_directories(mission_path_);
    }

    if(load_from_gpx_){
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

        publishPath();
        return;
    }

    if(preprocessed_map_){
        map_service_ = new MapService(map_name_);
    } else{
        ROS_WARN_STREAM("Handling non-preprocessed maps not implemented yet");
        ros::shutdown();
        //map_service_ = new MapService(extent,map_name(optional))
    }
    std::pair<OGRPoint,OGRPoint> map_extent = map_service_->getMapExtent();
    tree_ = new Quadtree(map_extent.first,map_extent.second,map_service_->getDataset(),map_name_,!preprocessed_map_);
    
    vessel_model_ = new ModelLibrary::Viknes830();
    search_alg_ = new HybridAStar(tree_,vessel_model_,map_service_,mission_name_);
    ROS_INFO_STREAM("Done initializing MissionPlanner");



}

void MissionPlanner::publishPath(){
    geometry_msgs::Pose wpt_msg;
    for(auto path_it = path_.begin(); path_it!=path_.end();path_it++){
        wpt_msg.position.x = (*path_it)->pose->x();
        wpt_msg.position.y = (*path_it)->pose->y();
        path_pub_.publish(wpt_msg);
        ros::Duration(0.01).sleep();
    }
}

void MissionPlanner::odomCb(const nav_msgs::Odometry& odom){
    latest_odom_ = odom;
}

void MissionPlanner::goalCb(const geometry_msgs::Pose& goal){
    search_alg_->setGoal(goal.position.x,goal.position.y,0);
}

bool MissionPlanner::search(usv_mission_planner::search::Request &req, usv_mission_planner::search::Response &res){
    ROS_INFO_STREAM("Start search in mission planner");
    if (req.use_odom){
        tf::Quaternion q;
        tf::quaternionMsgToTF(latest_odom_.pose.pose.orientation,q);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        search_alg_->setStart(latest_odom_.pose.pose.position.x,latest_odom_.pose.pose.position.y,yaw);
    } else{
        search_alg_->setStart(req.custom_start_lon,req.custom_start_lat,req.custom_start_heading);
    }
    search_alg_->setGoal(req.goal_lon,req.goal_lat,0);
    search_alg_->search();
    path_ = search_alg_->getPath();

    if(req.publish_path){
        publishPath();
    }
    savePath();

    res.success = true;
    return true;

}

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

MissionPlannerClient::MissionPlannerClient(ros::NodeHandle& nh): nh_(nh){
    search_client_ = nh_.serviceClient<usv_mission_planner::search>("SearchGlobalPath");
}

void MissionPlannerClient::searchFromOdom(double goal_lon, double goal_lat,bool publish_path){
    usv_mission_planner::search srv;
    srv.request.use_odom=true;
    srv.request.publish_path = publish_path;
    srv.request.goal_lon = goal_lon;
    srv.request.goal_lat = goal_lat;
    if(!search_client_.call(srv)) ROS_ERROR_STREAM("Search service call failed!");
}

void MissionPlannerClient::searchFromCustom(double start_lon,double start_lat, double start_heading, double goal_lon, double goal_lat, bool publish_path){
    usv_mission_planner::search srv;
    srv.request.use_odom=false;
    srv.request.publish_path = publish_path;
    srv.request.custom_start_lon = start_lon;
    srv.request.custom_start_lat = start_lat;
    srv.request.custom_start_heading = start_heading;
    srv.request.goal_lon = goal_lon;
    srv.request.goal_lat = goal_lat;
    ROS_INFO_STREAM("Calling search in MissionPlanner");
    if(!search_client_.call(srv)) ROS_ERROR_STREAM("Search service call failed!");
}

