#include "usv_mission_planner/astar.h"

AStar::AStar(GraphManager* gm, MapService* map_service,std::string mission_name):
gm_(gm), 
geod_(GeographicLib::Geodesic::WGS84()),
map_service_(map_service),
mission_name_(mission_name){
    search_id_ = 0;
}

/**
 * @brief Set search start vertex
 * 
 * @remark The exact (lon,lat) must exist in the graph managed by the GraphManager prior to calling this function!
 * 
 * @param lon Start longitude
 * @param lat Start latitude
 */
void AStar::setStart(double lon,double lat){
    StateVec start(lon,lat,0,0);
    gm_->getNearestVertex(&start,&v_start_);
}

/**
 * @brief Set search goal vertex
 * 
 * @remark The exact (lon,lat) must exist in the graph managed by the GraphManager prior to calling this function!
 * 
 * @param lon Goal longitude
 * @param lat Goal latitude
 */
void AStar::setGoal(double lon,double lat){
    StateVec goal(lon,lat,0,0);
    gm_->getNearestVertex(&goal,&v_goal_);
}

/**
 * @brief Search for a graph-optimal path.
 * 
 * @return true 
 * @return false 
 */
bool AStar::search(){
    ros::Time start = ros::Time::now();
    int id = generateSearchID();
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
        closed_.insert(current->id);
        //Early exit
        if (current==v_goal_){
            //std::cout << "Not following stored path" << std::endl;
            bool path_reconstructed = reconstructPath();
            updateLookupTable();
            saveDataContainers(id);
            search_early_exit_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
            return path_reconstructed;
        }
        
        if(followingStoredPath(current)){
            //std::cout << "Following stored path" << std::endl;
            bool path_reconstructed = reconstructPathFromLookup(current);
            updateLookupTable();
            saveDataContainers(id);
            search_sequence_match_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
            return path_reconstructed;
        }
        
        if(!ros::ok()){
            std::cout << "Stopping A* prematurely" << std::endl;
            saveDataContainers(generateSearchID());
            return false;
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
                if(closed_.find(next->id)!=closed_.end()) closed_.erase(next->id);
                came_from_[next]=current;
            }
        }
    }
}

/**
 * @brief Get the path stored in path_
 * 
 * @return std::vector<Vertex*> Path
 */
std::vector<Vertex*> AStar::getPath(){
    return path_;
}

/**
 * @brief Heuristic function for the A* Search. Approximates the true distance between to (lon,lat,0) points with GeographicLib
 * 
 * @param state_u Point A
 * @param state_v Point B
 * @return double Absolute distance between A and B [m]
 */
double AStar::heuristicDirect(const StateVec& state_u, const StateVec& state_v){
    ros::Time start = ros::Time::now();
    double distance;
    geod_.Inverse(state_u.y(),state_u.x(),state_v.y(),state_v.x(),distance);
    heuristic_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return abs(distance);
}

/**
 * @brief Based on the came_from_ map produced by search, fetch the path from start to goal.
 * 
 * @return true Path exists between start and goal
 * @return false Path does not exist between start and goal
 */
bool AStar::reconstructPath() {
    ros::Time start = ros::Time::now();
    path_.clear();
    path_length_ = 0;
    double spline_length = 0;
    Vertex* current = v_goal_;
    Vertex* next = nullptr;
    distance_lookup_table_.insert(std::make_pair(v_goal_->id,0));
    while (current != v_start_) {
        path_.push_back(current);
        auto came_from_it = came_from_.find(current);
        if(came_from_it==came_from_.end()){
            path_.clear();
            return false;
        }
        next = (*came_from_it).second;
        geod_.Inverse(next->state.y(),next->state.x(),current->state.y(), current->state.x(), spline_length);
        path_length_+=abs(spline_length);
        distance_lookup_table_.insert(std::make_pair(next->id,path_length_));
        current = next;
    }
    path_.push_back(v_start_); // optional
    std::reverse(path_.begin(), path_.end());
    reconstruct_full_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return true;
}

bool AStar::edgeInLookupTable(const Vertex* from, const Vertex* to){
    return (path_lookup_table_.find(from->id)!=path_lookup_table_.end() && (*path_lookup_table_.find(from->id)).second==to->id);
}

void AStar::updateLookupTable(){
    ros::Time start = ros::Time::now();
    for(int i=0; i<path_.size()-1; i++){
        path_lookup_table_[path_[i]->id]=path_[i+1]->id;
    }
    update_lookup_table_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
}

bool AStar::reconstructPathFromLookup(Vertex* v){
    //Knows optimal path from v to goal.
    ros::Time start = ros::Time::now();
    path_.clear();  
    Vertex* current = v;
    while (current != v_start_) {
        path_.push_back(current);
        auto came_from_it = came_from_.find(current);
        if(came_from_it==came_from_.end()){
            path_.clear();
            return false;
        }
        current = (*came_from_it).second;
    }
    path_.push_back(v_start_);
    std::reverse(path_.begin(), path_.end());

    //Reconstruct rest based on lookup
    current = v;
    while(current!=v_goal_){
        current = gm_->getVertex(path_lookup_table_[current->id]);
        path_.push_back(current);
    }
    path_.push_back(v_goal_);
    reconstruct_lookup_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return true;
}

bool AStar::followingStoredPath(Vertex* v){
    ros::Time start = ros::Time::now();
    Vertex* current = v;
    int necessary_match_sequence = 5;
    int i =0;
    match_sequence_.clear();
    while(i<necessary_match_sequence){
        match_sequence_.push_back(current);
        if(!edgeInLookupTable(came_from_[current],current) || !path_lookup_table_[came_from_[current]->id]==current->id){
            match_sequence_.clear();
            return false;
        }
        current = came_from_[current];
        i++;
    }
    match_sequence_.push_back(current);
    following_stored_path_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return true;
}

int AStar::generateSearchID(){
    return search_id_++;
}

/**
 * @brief Save data containers relevant for debugging/visualization
 * 
 */
void AStar::saveDataContainers(int search_id){
    ros::Time start = ros::Time::now();
    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/missions/"+mission_name_+"/astar/search/"+std::to_string(search_id)+"/");
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directories(path);
    }

    std::string found_path_path = path+"path.csv";
    std::string closed_file_path = path+"closed.csv";
    std::string came_from_file_path = path+"came_from.csv";
    std::string explored_file_path = path+"explored_file.csv";
    std::string frontier_file_path = path+"frontier.csv";
    std::string distance_lookup_table_path = path+"distance_lookup_table.csv";
    std::string match_sequence_path = path+"match_sequence.csv";

    std::ofstream closed_file(closed_file_path);
    closed_file << "lon,lat,psi,id\n";

    std::ofstream came_from_file(came_from_file_path);
    came_from_file << "lon_from,lat_from,psi_from,lon_to,lat_to,psi_to\n";

    std::ofstream explored_file(explored_file_path);
    explored_file << "lon,lat,psi,id\n";

    std::ofstream frontier_file(frontier_file_path);
    frontier_file << "lon,lat,psi,id\n";

    std::ofstream path_file(found_path_path);
    path_file << "id,lon,lat\n";

    std::ofstream distance_lookup_table_file(distance_lookup_table_path);
    distance_lookup_table_file << "id,distance_to_goal\n";

    std::ofstream match_sequence_file(match_sequence_path);
    match_sequence_file << "id,lon,lat\n";
    
    //std::cout << "Creating debug files" << std::endl;
    for(auto path_it=path_.begin(); path_it!=path_.end(); path_it++){
        path_file << (*path_it)->id <<","<<(*path_it)->state.x() << "," << (*path_it)->state.y() << "\n";
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
        Vertex* v = gm_->getVertex(*closed_it);
        closed_file<<v->state.x()<<","<<v->state.y()<<","<<v->state.w()<<","<<v->id<<"\n";
    }

    for(auto it = distance_lookup_table_.begin(); it!=distance_lookup_table_.end(); it++){
        distance_lookup_table_file << (*it).first<<","<<(*it).second<<"\n";
    }

    for(auto it = match_sequence_.begin(); it!=match_sequence_.end(); it++){
        match_sequence_file << (*it)->id << "," << (*it)->state.x() << "," << (*it)->state.y()<<"\n";
    }
    
    came_from_file.close();
    explored_file.close();
    closed_file.close();
    frontier_file.close();
    path_file.close();
    distance_lookup_table_file.close();
    match_sequence_file.close();
    save_data_contianer_times_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    //std::cout << "Debug files saved" << std::endl;
}

void AStar::dumpSearchBenchmark(){
    std::string path = ros::package::getPath("usv_mission_planner")+"/data/missions/"+mission_name_+"/astar/benchmark/";
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directories(path);
    }

    std::ofstream benchmark_file_time(path+"benchmark_time.csv");
    std::ofstream benchmark_file_misc(path+"benchmark_misc.csv");
    benchmark_file_time<<"name,times_run,total_time\n";
    benchmark_file_misc<<"name,value\n";



    ROS_INFO_STREAM("Search for path early exit " << search_early_exit_times_.size() << " times. Total time spent: " << std::accumulate(search_early_exit_times_.begin(),search_early_exit_times_.end(),0.0));
    ROS_INFO_STREAM("Search for path sequence match " << search_sequence_match_times_.size() << " times. Total time spent: " << std::accumulate(search_sequence_match_times_.begin(),search_sequence_match_times_.end(),0.0));
    ROS_INFO_STREAM("Update lookup table " << update_lookup_table_times_.size() << " times. Total time spent: " << std::accumulate(update_lookup_table_times_.begin(),update_lookup_table_times_.end(),0.0));
    ROS_INFO_STREAM("Check match sequence " << following_stored_path_times_.size() << " times. Total time spent: " << std::accumulate(following_stored_path_times_.begin(),following_stored_path_times_.end(),0.0));
    ROS_INFO_STREAM("Calculate heuristic " << heuristic_times_.size() << " times. Total time spent: " << std::accumulate(heuristic_times_.begin(),heuristic_times_.end(),0.0));
    ROS_INFO_STREAM("Reconstruct from lookup " << reconstruct_lookup_times_.size() << " times. Total time spent: " << std::accumulate(reconstruct_lookup_times_.begin(),reconstruct_lookup_times_.end(),0.0));
    ROS_INFO_STREAM("Reconstruct from scratch " << reconstruct_full_times_.size() << " times. Total time spent: " << std::accumulate(reconstruct_full_times_.begin(),reconstruct_full_times_.end(),0.0));
    ROS_INFO_STREAM("Save data containers " << save_data_contianer_times_.size() << " times. Total time spent: " << std::accumulate(save_data_contianer_times_.begin(),save_data_contianer_times_.end(),0.0));
    benchmark_file_time<<"search_early_exit"<<","<<search_early_exit_times_.size()<<","<<std::accumulate(search_early_exit_times_.begin(),search_early_exit_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"search_sequence_match"<<","<<search_sequence_match_times_.size()<<","<<std::accumulate(search_sequence_match_times_.begin(),search_sequence_match_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"update_lookup_table"<<","<<update_lookup_table_times_.size()<<","<<std::accumulate(update_lookup_table_times_.begin(),update_lookup_table_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"check_following_stored_path"<<","<<following_stored_path_times_.size()<<","<<std::accumulate(following_stored_path_times_.begin(),following_stored_path_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"calculate_heuristic"<<","<<heuristic_times_.size()<<","<<std::accumulate(heuristic_times_.begin(),heuristic_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"reconstruct_lookup"<<","<<reconstruct_lookup_times_.size()<<","<<std::accumulate(reconstruct_lookup_times_.begin(),reconstruct_lookup_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"reconstruct_full"<<","<<reconstruct_full_times_.size()<<","<<std::accumulate(reconstruct_full_times_.begin(),reconstruct_full_times_.end(),0.0)<<"\n";
    benchmark_file_time<<"save_data_contianers"<<","<<save_data_contianer_times_.size()<<","<<std::accumulate(save_data_contianer_times_.begin(),save_data_contianer_times_.end(),0.0)<<"\n";

    //Benchmark containers
    int id = 0;
    std::ofstream search_early_exit_file(path+"early_exit.csv");
    search_early_exit_file << "time\n";

    std::ofstream search_sequence_match_file(path+"sequence_match.csv");
    search_sequence_match_file << "time\n";

    for(auto it = search_early_exit_times_.begin(); it!= search_early_exit_times_.end(); it++){
        search_early_exit_file<<(*it)<<"\n";
    }

    for(auto it=search_sequence_match_times_.begin(); it!= search_sequence_match_times_.end(); it++){
        search_sequence_match_file<<(*it)<<"\n";
    }

    search_early_exit_file.close();
    search_sequence_match_file.close();
}

AStarROS::AStarROS(ros::NodeHandle& nh, GraphManager* gm, MapService* map_service):
nh_(nh),
AStar(gm,map_service,"dummy"){
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