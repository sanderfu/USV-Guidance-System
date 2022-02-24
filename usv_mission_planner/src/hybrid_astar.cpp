#include "usv_mission_planner/hybrid_astar.h"

HybridAStar::HybridAStar(Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapServiceClient* map_client):
tree_(tree),
vessel_model_(vessel_model),
geod_(GeographicLib::Geodesic::WGS84()),
map_client_(map_client),
grid_search_alg_(new AStar(tree->getGraphManager())){

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
    ros::Time start_search = ros::Time::now();
    frontier_.put(v_start_,0);
    came_from_[v_start_] = v_start_;
    cost_so_far_[v_start_] = 0;

    std::vector<double> heading_candidates = {-M_PI/6,0,M_PI/6};
    std::vector<double> collision_time;
    std::vector<double> leaf_time;
    std::vector<double> enu_to_wgs_time;
    std::vector<double> simulate_time;
    std::vector<double> heuristic_time;
    Region* current_region;
    Region* candidate_region;

    while(!frontier_.empty()){
        extendedVertex* current = frontier_.get();
        closed_.push_back(current);
        current_region = tree_->getLeafRegionContaining(current->pose->x(),current->pose->y());

        //Early exit
        if(getDistance(v_goal_->pose,current->pose)<100){
            ROS_INFO_STREAM("Early exit");
            v_close_ = current;
            break;
        }

        if(!ros::ok()){
            std::cout << "Stopping prematurely" << std::endl;
            saveDataContainers();
            exit(1);
        }

        //Calculate vertex candidates
        for(auto heading_candidate_it=heading_candidates.begin();heading_candidate_it!=heading_candidates.end();heading_candidate_it++){
            Eigen::Vector3d pos_wgs(current->pose->x(),current->pose->y(),0);
            Eigen::Vector3d pos_enu;
            geo_converter_.convert("WGS84",pos_wgs,"start_enu",&pos_enu);
            state_type candidate_state = {pos_enu.x(),pos_enu.y(),current->pose->w(),current->twist->x(),current->twist->y(),current->twist->z()};
            ros::Time start_sim = ros::Time::now();
            ModelLibrary::simulatedHorizon sim_hor = vessel_model_->simulateHorizonAdaptive(candidate_state,3,*heading_candidate_it+current->pose->w(),60);
            ros::Time end_sim = ros::Time::now();
            simulate_time.push_back(ros::Duration(end_sim-start_sim).toSec());

            Eigen::Vector3d candidate_enu(candidate_state[0],candidate_state[1],0);
            Eigen::Vector3d candidate_wgs;
            ros::Time start_conv = ros::Time::now();
            geo_converter_.convert("start_enu",candidate_enu,"WGS84",&candidate_wgs);
            ros::Time end_conv = ros::Time::now();
            enu_to_wgs_time.push_back(ros::Duration(end_conv-start_conv).toSec());

            //If in same region as last time, cant possibly be collision
            ros::Time start_leaf_search = ros::Time::now();
            candidate_region = tree_->getLeafRegionContaining(candidate_wgs.x(),candidate_wgs.y());
            ros::Time end_leaf_search = ros::Time::now();
            leaf_time.push_back(ros::Duration(end_leaf_search-start_leaf_search).toSec());

            //If candidate region is nullptr, definitely path into land, no need to check for detailed collision
            if(candidate_region==nullptr){
                points_outside_quadtree_.push_back(std::make_pair(candidate_wgs.x(),candidate_wgs.y()));
                continue;
            }

            bool spline_collision=false;
            if (candidate_region!=current_region){
                //Check for collision, if exists do not add state to open vertices
                ros::Time start_collision = ros::Time::now();
                if(collision(sim_hor)) spline_collision=true;
                ros::Time end_collision = ros::Time::now();
                collision_time.push_back(ros::Duration(end_collision-start_collision).toSec());
                if(spline_collision) continue;
            }

            state_type candidate_state_wgs = {candidate_wgs.x(),candidate_wgs.y(),candidate_state[2],candidate_state[3],candidate_state[4],candidate_state[5]};
            extendedVertex* next = new extendedVertex(generateVertexID(),candidate_state_wgs);


            //If vertex sufficiently simiar has been closed, go to next candidate
            double closed_distance_check;
            bool next_closed =false;
            for(auto closed_it = closed_.begin(); closed_it!=closed_.end(); closed_it++){
                geod_.Inverse((*closed_it)->pose->y(),(*closed_it)->pose->x(),next->pose->y(),next->pose->x(),closed_distance_check);
                if(closed_distance_check<25){
                    delete next;
                    next_closed=true;
                    break;
                }
            }
            if (next_closed){
                continue;
            }


            double new_cost = cost_so_far_[current] + getDistance(current->pose,next->pose);

            //Find most similar node prdviously explored (if exists)
            double distance_check;
            bool explored = false;
            for(auto vertex_it=cost_so_far_.begin(); vertex_it!=cost_so_far_.end();vertex_it++){
                geod_.Inverse((*vertex_it).first->pose->y(),(*vertex_it).first->pose->x(),next->pose->y(),next->pose->x(),distance_check);
                if (distance_check<25){// && abs(SSA((*vertex_it).first->pose->w()-next->pose->w()))<M_PI/9){
                    explored=true;
                    delete next;
                    next = (*vertex_it).first;
                }
            }

            if(!explored || new_cost<cost_so_far_[next]){
                cost_so_far_[next]=new_cost;
                ros::Time start_heuristic = ros::Time::now();
                double priority = new_cost + std::max(getDistance(next->pose,v_goal_->pose),getGridDistance(next->pose,v_goal_->pose)); //+ heuristic(next->state,v_goal_->state);
                ros::Time end_heuristic = ros::Time::now();
                heuristic_time.push_back(ros::Duration(end_heuristic-start_heuristic).toSec());
                frontier_.put(next,priority);
                came_from_[next]=current;
            }
        }
    }
    path_ = reconstructPath();
    ros::Time end_search = ros::Time::now();
    saveDataContainers();

    //Benchmark info:
    ROS_INFO_STREAM("Search took: " << ros::Duration(end_search-start_search).toSec());
    ROS_INFO_STREAM("Check for leaf " << leaf_time.size() << " times. Total time spent: " << std::accumulate(leaf_time.begin(),leaf_time.end(),0.0));
    ROS_INFO_STREAM("Check for collision " << collision_time.size() << " times. Total time spent: " << std::accumulate(collision_time.begin(),collision_time.end(),0.0));
    ROS_INFO_STREAM("Simulate " << simulate_time.size() << " times. Total time spent: " << std::accumulate(simulate_time.begin(),simulate_time.end(),0.0));
    ROS_INFO_STREAM("ENU to WGS " << enu_to_wgs_time.size() << " times. Total time spent: " << std::accumulate(enu_to_wgs_time.begin(),enu_to_wgs_time.end(),0.0));
    ROS_INFO_STREAM("Calculate heuristic " << heuristic_time.size() << " times. Total time spent: " << std::accumulate(heuristic_time.begin(),heuristic_time.end(),0.0));
}

int HybridAStar::generateVertexID(){
    return vertex_id_++;
}

double HybridAStar::getDistance(StateVec* u, StateVec* v){
    double distance;
    geod_.Inverse(u->y(),u->x(),v->y(),v->x(),distance);
    return abs(distance);
}

double HybridAStar::getGridDistance(StateVec* u, StateVec* v){
    //Check if roughly this start->goal config has been checked before

    Region* current = tree_->getLeafRegionContaining(u->x(),u->y());
    auto grid_lookup_it = grid_distance_lookup_.find(current);
    if (grid_lookup_it!=grid_distance_lookup_.end()){
        return (*grid_lookup_it).second;
    }

    tree_->setStart(current->centroid_.getX(),current->centroid_.getY());
    tree_->setGoal(v->x(),v->y());



    grid_search_alg_->setStart(current->centroid_.getX(),current->centroid_.getY());
    grid_search_alg_->setGoal(v->x(),v->y());
    grid_search_alg_->search();
    std::vector<Vertex*> shortest_path = grid_search_alg_->getPath();
    double spline_distance=0;
    double total_distance=0;
    for(int i=0; i<shortest_path.size()-1; i++){
        geod_.Inverse(shortest_path[i]->state.y(),shortest_path[i]->state.x(),shortest_path[i+1]->state.y(),shortest_path[i+1]->state.x(),spline_distance);
        total_distance+=spline_distance;
    }
    double distance_u_v = 0;
    double distance_centroid_v = 0;
    geod_.Inverse(u->y(),u->x(),v->y(),v->x(),distance_u_v);
    geod_.Inverse(current->centroid_.getY(),current->centroid_.getX(),v->y(),v->x(),distance_centroid_v);
    double difference = abs(distance_u_v)-abs(distance_centroid_v);
    grid_distance_lookup_.insert(std::make_pair(current,total_distance+difference));
    return total_distance+difference;
}

bool HybridAStar::collision(ModelLibrary::simulatedHorizon& sim_hor){
    //Check for collision
    spline_.empty();
    Eigen::Vector3d point_local;
    Eigen::Vector3d point_global;
    for(auto hor_it = sim_hor.state.begin(); hor_it!=sim_hor.state.end();hor_it++){
        point_local(0) = hor_it->at(0);
        point_local(1) = hor_it->at(1);
        point_local(2) = 0;
        geo_converter_.convert("start_enu",point_local,"WGS84",&point_global);
        spline_.addPoint(point_global(0),point_global(1));
    }
    
    if (map_client_->collision(&spline_)){
        //Path from current vertex to candidate collides with land, do not add the candidate to the open vertex list
        return true;
    } else{
        return false;
    }
}

std::vector<extendedVertex*> HybridAStar::reconstructPath() {
    std::vector<extendedVertex*> path;
    extendedVertex* current = v_close_;
    while (current != v_start_) {
        path.push_back(current);
        current = came_from_[current];
    }
    path.push_back(v_start_); // optional
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<extendedVertex*> HybridAStar::getPath(){
    return path_;
}

void HybridAStar::saveDataContainers(){
    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/debug/hybrid_astar/");

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
    closed_file << "lon,lat,psi,u,v,r,id\n";

    std::ofstream came_from_file(came_from_file_path);
    came_from_file << "lon_from,lat_from,psi_from,lon_to,lat_to,psi_to\n";

    std::ofstream explored_file(explored_file_path);
    explored_file << "lon,lat,psi,u,v,r,id\n";

    std::ofstream frontier_file(frontier_file_path);
    frontier_file << "lon,lat,psi,u,v,r,id\n";

    std::ofstream path_file(found_path_path);
    path_file << "lon,lat\n";

    std::ofstream points_outside_quadtree_file(points_outside_quadtree_path);
    points_outside_quadtree_file << "lon,lat\n";

    for(auto path_it=path_.begin(); path_it!=path_.end(); path_it++){
        path_file << (*path_it)->pose->x() << "," << (*path_it)->pose->y() << std::endl;
    }

    std::cout << "Creating debug files" << std::endl;
    for (auto came_from_it = came_from_.begin(); came_from_it!=came_from_.end(); came_from_it++){
        extendedVertex* from = (*came_from_it).second;
        extendedVertex* to = (*came_from_it).first;
        came_from_file<<from->pose->x()<<","<<from->pose->y()<<","<<from->pose->w()<<","<<to->pose->x()<<","<<to->pose->y()<<","<<to->pose->w()<<"\n";
        explored_file<<to->pose->x()<<","<<to->pose->y()<<","<<to->pose->w()<<","<<to->twist->x()<<","<<to->twist->y()<<","<<to->twist->z()<<","<<to->id_<<"\n";
    }

    while(!frontier_.empty()){
        extendedVertex* v = frontier_.get();
        frontier_file<<v->pose->x()<<","<<v->pose->y()<<","<<v->pose->w()<<","<<v->twist->x()<<","<<v->twist->y()<<","<<v->twist->z()<<","<<v->id_<<"\n";
    }

    for(auto closed_it=closed_.begin(); closed_it!=closed_.end();closed_it++){
        extendedVertex* v = (*closed_it);
        closed_file<<v->pose->x()<<","<<v->pose->y()<<","<<v->pose->w()<<","<<v->twist->x()<<","<<v->twist->y()<<","<<v->twist->z()<<","<<v->id_<<"\n";
    }

    for(auto outside_it=points_outside_quadtree_.begin(); outside_it!=points_outside_quadtree_.end();outside_it++){
        points_outside_quadtree_file<<outside_it->first<<","<<outside_it->second<<"\n";
    }
    came_from_file.close();
    explored_file.close();
    closed_file.close();
    frontier_file.close();
    points_outside_quadtree_file.close();
    std::cout << "Debug files saved" << std::endl;
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