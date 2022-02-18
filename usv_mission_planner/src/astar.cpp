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
    std::unordered_map<Vertex*, Vertex*> came_from;
    std::unordered_map<Vertex*, double> cost_so_far;
    PriorityQueue<Vertex*,double> frontier;
    frontier.put(v_start_,0);
    came_from[v_start_] = v_start_;
    cost_so_far[v_start_] = 0;

    while(!frontier.empty()){
        Vertex* current = frontier.get();

        //Early exit
        if (current==v_goal_){
            ROS_INFO_STREAM("Early exit");
            break;
        }

        std::vector<Vertex*> neighbors;
        gm_->getConnectedNeighbors(current->id,neighbors);
        for (Vertex* next : neighbors){
            if (next==current){
                continue;
            }
            
            double new_cost = cost_so_far[current] + gm_->getEdgeWeight(current->id,next->id);

            if(cost_so_far.find(next) == cost_so_far.end() || new_cost<cost_so_far[next]){
                cost_so_far[next]=new_cost;
                double priority = new_cost + heuristic(next->state,v_goal_->state);
                frontier.put(next,priority);
                came_from[next]=current;
            }
        }
        
    }
    path_ = reconstructPath(came_from);
    return true;
}

std::vector<Vertex*> AStar::getPath(){
    return path_;
}

double AStar::heuristic(const StateVec& state_u, const StateVec& state_v){
    double distance;
    geod_.Inverse(state_u.y(),state_u.x(),state_v.y(),state_v.x(),distance);
    return abs(distance);
}

std::vector<Vertex*> AStar::reconstructPath(std::unordered_map<Vertex*, Vertex*>& came_from) {
    std::vector<Vertex*> path;
    Vertex* current = v_goal_;
    while (current != v_start_) {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(v_start_); // optional
    std::reverse(path.begin(), path.end());
    return path;
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