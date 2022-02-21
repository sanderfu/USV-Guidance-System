#include "usv_mission_planner/hybrid_astar.h"

HybridAStar::HybridAStar(Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapServiceClient* map_client):
tree_(tree),
vessel_model_(vessel_model),
geod_(GeographicLib::Geodesic::WGS84()),
map_client_(map_client){

    //Set up geo converter
    geo_converter_.addFrameByEPSG("WGS84",4326);
}

void HybridAStar::setStart(double lon, double lat, double yaw){
    geo_converter_.addFrameByENUOrigin("start_enu",lat,lon,0);
    state_type state = {lon,lat,yaw,0,0,0};
    v_start_ = new extendedVertex(generateVertexID(),state);
}

void HybridAStar::setGoal(double lon, double lat, double yaw){
    state_type state = {lon,lat,yaw,0,0,0};
    v_goal_ = new extendedVertex(generateVertexID(),state);
}

void HybridAStar::search(){
    std::unordered_map<extendedVertex*, extendedVertex*> came_from;
    std::unordered_map<extendedVertex*, double> cost_so_far;
    //FlexiblePriorityQueue<extendedVertex> frontier;
    PriorityQueue<extendedVertex*,double> frontier;
    frontier.put(v_start_,0);
    came_from[v_start_] = v_start_;
    cost_so_far[v_start_] = 0;

    std::vector<double> heading_candidates = {-M_PI/3,-M_PI/6,0,-M_PI/6,M_PI/3};

    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/debug/hybrid_astar/");

    std::string debug_file_path = path+"debug.csv";

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::ofstream debug_file(debug_file_path);
    debug_file << "lon,lat,psi,u,v,r,id\n";
    while(!frontier.empty()){
        extendedVertex* current = frontier.get();
        debug_file << current->pose->x() << "," << current->pose->y() << "," << current->pose->w() << "," << current->twist->x() << "," << current->twist->y() << "," << current->twist->z() << "," << current->id_ << "\n";
        //std::cout << "Distance to goal from current: " << getDistance(current->pose,v_goal_->pose) << std::endl;
        //std::cout << "Continue with enter" << std::endl;
        //std::cin.get();


        //Early exit
        if(getDistance(v_goal_->pose,current->pose)<100){
            ROS_INFO_STREAM("Early exit");
            v_close_ = current;
            break;
        }

        if(!ros::ok()){
            ROS_WARN_STREAM("Stopping prematurely");
            debug_file.close();
            exit(1);
        }

        //Calculate vertex candidates
        for(auto heading_candidate_it=heading_candidates.begin();heading_candidate_it!=heading_candidates.end();heading_candidate_it++){
            Eigen::Vector3d pos_wgs(current->pose->x(),current->pose->y(),0);
            Eigen::Vector3d pos_enu;
            geo_converter_.convert("WGS84",pos_wgs,"start_enu",&pos_enu);
            state_type candidate_state = {pos_enu.x(),pos_enu.y(),current->pose->w(),current->twist->x(),current->twist->y(),current->twist->z()};

            ModelLibrary::simulatedHorizon sim_hor = vessel_model_->simulateHorizonAdaptive(candidate_state,3,*heading_candidate_it+current->pose->w(),30);

            //Check for collision, if exists do not add state to open vertices
            if(collision(sim_hor)) continue;

            Eigen::Vector3d candidate_enu(candidate_state[0],candidate_state[1],0);
            Eigen::Vector3d candidate_wgs;
            geo_converter_.convert("start_enu",candidate_enu,"WGS84",&candidate_wgs);

            state_type candidate_state_wgs = {candidate_wgs.x(),candidate_wgs.y(),candidate_state[2],candidate_state[3],candidate_state[4],candidate_state[5]};
            extendedVertex* next = new extendedVertex(generateVertexID(),candidate_state_wgs);

            double new_cost = cost_so_far[current] + getDistance(current->pose,next->pose);

            //Find most similar node prdviously explored (if exists)
            double distance_check;
            bool explored = false;
            for(auto vertex_it=cost_so_far.begin(); vertex_it!=cost_so_far.end();vertex_it++){
                geod_.Inverse((*vertex_it).first->pose->y(),(*vertex_it).first->pose->x(),next->pose->y(),next->pose->x(),distance_check);
                if (distance_check<25){ //&& abs(SSA((*vertex_it).first->pose->w()-next->pose->w()))<M_PI/12){
                    explored=true;
                    delete next;
                    next = (*vertex_it).first;
                }
            }

            if(!explored || new_cost<cost_so_far[next]){
                cost_so_far[next]=new_cost;
                double priority = new_cost + getDistance(next->pose,v_goal_->pose); //+ heuristic(next->state,v_goal_->state);

                //Check if exists in priority queue, if new pririty better add
                /*
                double distance_check;
                std::map<extendedVertex*,pri_key_t>* latest_key_ = frontier.getCurrentPriority();
                for(auto key_it = latest_key_->begin();key_it!=latest_key_->end();key_it++){
                    geod_.Inverse((*key_it).first->pose->y(),(*key_it).first->pose->x(),next->pose->y(),next->pose->x(),distance_check);
                    if (distance_check<25){
                        if (priority<(*key_it).second.first){
                            ROS_INFO_STREAM("New candidate better");
                            came_from[(*key_it).first]=current;
                            frontier.push((*key_it).first,std::make_pair(priority,0));
                            delete next;
                            continue;
                        }
                    }
                }
                */

                frontier.put(next,priority);
                came_from[next]=current;
            }
        }
    }
    path_ = reconstructPath(came_from);

    std::string debug_path_file_path = path+"path.csv";

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::ofstream debug_path_file(debug_path_file_path);
    debug_path_file << "lon,lat\n";

    //Save path to csv for debug
    for (auto path_it=path_.begin(); path_it!=path_.end(); path_it++){
        debug_path_file << (*path_it)->pose->x() << "," << (*path_it)->pose->y() << std::endl;
    }
    debug_path_file.close();
}

int HybridAStar::generateVertexID(){
    return vertex_id_++;
}

double HybridAStar::getDistance(StateVec* u, StateVec* v){
    double distance;
    geod_.Inverse(u->y(),u->x(),v->y(),v->x(),distance);
    return abs(distance);
}

bool HybridAStar::collision(ModelLibrary::simulatedHorizon& sim_hor){
    //Check for collision
    OGRLineString path;
    Eigen::Vector3d point_local;
    Eigen::Vector3d point_global;
    for(auto hor_it = sim_hor.state.begin(); hor_it!=sim_hor.state.end();hor_it++){
        point_local(0) = hor_it->at(0);
        point_local(1) = hor_it->at(1);
        point_local(2) = 0;
        geo_converter_.convert("start_enu",point_local,"WGS84",&point_global);
        path.addPoint(point_global(0),point_global(1));
    }

    if (map_client_->collision(&path)){
        //Path from current vertex to candidate collides with land, do not add the candidate to the open vertex list
        return true;
    } else{
        return false;
    }
}

std::vector<extendedVertex*> HybridAStar::reconstructPath(std::unordered_map<extendedVertex*, extendedVertex*>& came_from) {
    std::vector<extendedVertex*> path;
    extendedVertex* current = v_close_;
    while (current != v_start_) {
        path.push_back(current);
        current = came_from[current];
    }
    path.push_back(v_start_); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<extendedVertex*> HybridAStar::getPath(){
    return path_;
}

HybridAStarROS::HybridAStarROS(ros::NodeHandle& nh, Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapServiceClient* map_client):
nh_(nh),
HybridAStar(tree,vessel_model,map_client){
    path_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/hybrid_astar/visual_path",1,true);

    geo_converter_.addFrameByEPSG("WGS84",4326);
    geo_converter_.addFrameByENUOrigin("global_enu",40.5612,-73.9761,0);

    initializeMarkers();
}

void HybridAStarROS::initializeMarkers(){
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

void HybridAStarROS::addVisualPath(){
    geometry_msgs::Point point;
    for (auto path_it=path_.begin(); path_it!=path_.end();path_it++){
        if(path_it>path_.begin()+1){
            path_marker_.points.push_back(point);
        }
        Eigen::Vector3d u_wgs_eig((*path_it)->pose->x(),(*path_it)->pose->y(),0);
        Eigen::Vector3d u_enu_eig;
        geo_converter_.convert("WGS84",u_wgs_eig,"global_enu",&u_enu_eig);
        point.x = u_enu_eig.x();
        point.y = u_enu_eig.y();
        path_marker_.points.push_back(point);
    }
    publishVisualPath();
}

void HybridAStarROS::publishVisualPath(){
    path_marker_pub_.publish(path_marker_);
}

void HybridAStarROS::visualize(){
    addVisualPath();
}