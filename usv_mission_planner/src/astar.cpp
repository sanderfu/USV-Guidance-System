#include "usv_mission_planner/astar.h"

AStar::AStar(GraphManager* gm):gm_(gm), geod_(GeographicLib::Geodesic::WGS84()){}

void AStar::setStart(double lon,double lat){
    StateVec start(lon,lat,0,0);
    gm_->getNearestVertex(&start,&v_start_);
}

void AStar::setGoal(double lon,double lat){
    StateVec goal(lon,lat,0,0);
    gm_->getNearestVertex(&goal,&v_goal_);
}

bool AStar::search(){
    while(!frontier_.empty()){
        frontier_.get();
    }
    came_from_.clear();
    cost_so_far_.clear();
    closed_.clear();

    frontier_.put(v_start_,0);
    came_from_[v_start_] = v_start_;
    cost_so_far_[v_start_] = 0;

    while(!frontier_.empty()){
        Vertex* current = frontier_.get();
        closed_.push_back(current);
        //Early exit
        if (current==v_goal_){
            //ROS_INFO_STREAM("Early exit");
            break;
        }

        if(!ros::ok()){
            std::cout << "Stopping A* prematurely" << std::endl;
            saveDataContainers();
            exit(1);
        }

        std::vector<Vertex*> neighbors;
        gm_->getConnectedNeighbors(current->id,neighbors);
        for (Vertex* next : neighbors){
            if (next==current){
                continue;
            }
            double new_cost = cost_so_far_[current] + gm_->getEdgeWeight(current->id,next->id);

            if(cost_so_far_.find(next) == cost_so_far_.end() || new_cost<cost_so_far_[next]){
                cost_so_far_[next]=new_cost;
                double priority = cost_so_far_[next] + heuristicDirect(next->state,v_goal_->state);
                frontier_.put(next,priority);
                came_from_[next]=current;
            }
        }
        
    }
    return reconstructPath();
}

std::vector<Vertex*> AStar::getPath(){
    return path_;
}

double AStar::heuristicDirect(const StateVec& state_u, const StateVec& state_v){
    double distance;
    geod_.Inverse(state_u.y(),state_u.x(),state_v.y(),state_v.x(),distance);
    return abs(distance);
}

double AStar::heuristicDiagonal(const StateVec& state_u, const StateVec& state_v){
    double dx, dy;
    geod_.Inverse(state_u.y(),state_u.x(),state_u.y(),state_v.x(),dx);
    dx = abs(dx);
    geod_.Inverse(state_u.y(),state_u.x(),state_v.y(),state_u.x(),dy);
    dy = abs(dy);
    return dx+dy-std::min(dx,dy);
}

bool AStar::reconstructPath() {
    path_.clear();
    Vertex* current = v_goal_;
    while (current != v_start_) {
        path_.push_back(current);
        auto came_from_it = came_from_.find(current);
        if(came_from_it==came_from_.end()){
            path_.clear();
            return false;
        }
        current = (*came_from_it).second;
    }
    path_.push_back(v_start_); // optional
    std::reverse(path_.begin(), path_.end());
    return true;
}

void AStar::saveDataContainers(){
    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/debug/astar/");

    std::string found_path_path = path+"path.csv";
    std::string closed_file_path = path+"closed.csv";
    std::string came_from_file_path = path+"came_from.csv";
    std::string explored_file_path = path+"explored_file.csv";
    std::string frontier_file_path = path+"frontier.csv";
    std::string points_outside_quadtree_path = path + "outside_quadtree.csv";

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::ofstream closed_file(closed_file_path);
    closed_file << "lon,lat,psi,id\n";

    std::ofstream came_from_file(came_from_file_path);
    came_from_file << "lon_from,lat_from,psi_from,lon_to,lat_to,psi_to\n";

    std::ofstream explored_file(explored_file_path);
    explored_file << "lon,lat,psi,id\n";

    std::ofstream frontier_file(frontier_file_path);
    frontier_file << "lon,lat,psi,id\n";

    std::ofstream path_file(found_path_path);
    path_file << "lon,lat\n";

    
    std::cout << "Creating debug files" << std::endl;
    for(auto path_it=path_.begin(); path_it!=path_.end(); path_it++){
        path_file << (*path_it)->state.x() << "," << (*path_it)->state.y() << "\n";
    }

    for (auto came_from_it = came_from_.begin(); came_from_it!=came_from_.end(); came_from_it++){
        Vertex* from = (*came_from_it).second;
        Vertex* to = (*came_from_it).first;
        came_from_file<<from->state.x()<<","<<from->state.y()<<","<<from->state.w()<<","<<to->state.x()<<","<<to->state.y()<<","<<to->state.w()<<"\n";
        explored_file<<to->state.x()<<","<<to->state.y()<<","<<to->state.w()<<","<<to->id<<"\n";
    }

    while(!frontier_.empty()){
        Vertex* v = frontier_.get();
        frontier_file<<v->state.x()<<","<<v->state.y()<<","<<v->state.w()<<","<<v->id<<"\n";
    }
    

    for(auto closed_it=closed_.begin(); closed_it!=closed_.end();closed_it++){
        Vertex* v = (*closed_it);
        closed_file<<v->state.x()<<","<<v->state.y()<<","<<v->state.w()<<","<<v->id<<"\n";
    }
    
    came_from_file.close();
    explored_file.close();
    closed_file.close();
    frontier_file.close();
    path_file.close();
    std::cout << "Debug files saved" << std::endl;
}

AStarROS::AStarROS(ros::NodeHandle& nh, GraphManager* gm):
nh_(nh),
AStar(gm){
    path_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/astar/visual_path",1,true);

    geo_converter_.addFrameByEPSG("WGS84",4326);
    geo_converter_.addFrameByENUOrigin("global_enu",40.5612,-73.9761,0);

    initializeMarkers();
}

void AStarROS::initializeMarkers(){
    path_marker_.header.frame_id = "map";
    path_marker_.header.stamp = ros::Time::now();
    path_marker_.type = visualization_msgs::Marker::LINE_LIST;
    path_marker_.action = visualization_msgs::Marker::ADD;
    path_marker_.lifetime = ros::Duration();

    path_marker_.pose.position.x = 0.0;
    path_marker_.pose.position.y = 0.0;
    path_marker_.pose.position.z = 0.0;
    
    path_marker_.color.r = 0.5f;
    path_marker_.color.g = 0.0f;
    path_marker_.color.b = 0.5f;
    path_marker_.color.a = 1.0f;

    // Scale unit is meters
    path_marker_.scale.x = 1.0;
    path_marker_.scale.y = 1.0;
    path_marker_.scale.z = 1.0;

    path_marker_.pose.orientation.w = 1.0;
    path_marker_.id = 0;
}

void AStarROS::addVisualPath(){
    geometry_msgs::Point point;
    for (auto path_it=path_.begin(); path_it!=path_.end();path_it++){
        if(path_it>path_.begin()+1){
            path_marker_.points.push_back(point);
        }
        Eigen::Vector3d u_wgs_eig((*path_it)->state.x(),(*path_it)->state.y(),0);
        Eigen::Vector3d u_enu_eig;
        geo_converter_.convert("WGS84",u_wgs_eig,"global_enu",&u_enu_eig);
        point.x = u_enu_eig.x();
        point.y = u_enu_eig.y();
        path_marker_.points.push_back(point);
    }
    publishVisualPath();
}

void AStarROS::publishVisualPath(){
    path_marker_pub_.publish(path_marker_);
}

void AStarROS::visualize(){
    addVisualPath();
}