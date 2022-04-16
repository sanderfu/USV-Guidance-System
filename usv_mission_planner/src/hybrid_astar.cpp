#include "usv_mission_planner/hybrid_astar.h"
/**
 * @brief Construct a new Hybrid A Star object
 * 
 * @param tree A Quadtree used for focusing the heuristic
 * @param vessel_model A vessel model used for simulation.
 * @param map_service A map service to query collision and voronoi field values. 
 * @param mission_name The name of the mission. Relevant for data storage.
 */
HybridAStar::HybridAStar(Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapService* map_service, std::string mission_name):
tree_(tree),
vessel_model_(vessel_model),
geod_(GeographicLib::Geodesic::WGS84()),
map_service_(map_service),
grid_search_alg_(new AStar(tree->getGraphManager(),map_service_,mission_name)){
    geo_converter_.addFrameByEPSG("WGS84",4326);

    //Load parameters
    bool parameter_load_error = false;
    if(!ros::param::get("hybrid_a_star/search_pruning/prune_radius_explored",prune_radius_explored_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/search_pruning/prune_radius_closed",prune_radius_closed_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/static_simulation_time/default_sim_time",default_sim_time_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/adaptive_simulation_time/enable_adaptive_sim_time",enable_adaptive_sim_time_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/adaptive_simulation_time/underway_sim_time_minimum",underway_sim_time_minimum_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/adaptive_simulation_time/approach_sim_time_scaling",approach_sim_time_scaling_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/adaptive_simulation_time/approach_sim_time_minimum",approach_sim_time_minimum_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/heuristic/voronoi_field_cost_weight",voronoi_field_cost_weight_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/heuristic/distance_scaling_factor",distance_scaling_factor_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/mission_data/save_search_data",save_search_data_)) parameter_load_error = true;
    if(!ros::param::get("hybrid_a_star/mission_data/save_benchmark_data",save_benchmark_data_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }

    mission_name_ = mission_name;
}   

/**
 * @brief Set the start pose for the search
 * 
 * @param lon Star longitude
 * @param lat Starg latitude
 * @param yaw Start heading
 */
void HybridAStar::setStart(double lon, double lat, double yaw){
    geo_converter_.addFrameByENUOrigin("start_enu",lat,lon,0);
    state_type state = {lon,lat,yaw,0,0,0};
    v_start_ = new extendedVertex(generateVertexID(),state);
}

/**
 * @brief Set the goal pose for the search
 * 
 * @remark The heading is neglected for the time being
 * 
 * @param lon 
 * @param lat 
 * @param yaw 
 */
void HybridAStar::setGoal(double lon, double lat, double yaw){
    state_type state = {lon,lat,yaw,0,0,0};
    v_goal_ = new extendedVertex(generateVertexID(),state);
}

void HybridAStar::setMissionName(std::string mission_name){
    mission_name_ = mission_name;
    grid_search_alg_->setMissionName(mission_name);
}

/**
 * @brief Clear all data containers. 
 * Helping function to avoid leftover data whenever more than one search is requested form the same object.
 * 
 */
void HybridAStar::clear(){
    came_from_.clear();
    cost_so_far_.clear();
    closed_.clear();
    grid_distance_lookup_.clear();
    while(!frontier_.empty()) frontier_.get();

    collision_time_.clear();
    leaf_time_.clear();
    simulate_time_.clear();
    heuristic_time_.clear();
    calc_sim_time_.clear();
}

/**
 * @brief Find a feasible, optimized path fromt he start to the goal pose.
 * 
 */
void HybridAStar::search(){
    clear();
    start_search_ = ros::Time::now();
    frontier_.put(v_start_,0);
    came_from_[v_start_] = v_start_;
    cost_so_far_[v_start_] = 0;

    std::vector<double> heading_candidates = {-M_PI/6,-M_PI/9,0,M_PI/9,M_PI/6};
    Region* current_region;

    while(!frontier_.empty()){
        extendedVertex* current = frontier_.get();
        ros::Time start_leaf_search = ros::Time::now();
        Region* current_region = tree_->getLeafRegionContaining(current->pose->x(),current->pose->y());
        ros::Time end_leaf_search = ros::Time::now();
        leaf_time_.push_back(ros::Duration(end_leaf_search-start_leaf_search).toSec());

        closed_.push_back(current);

        start_leaf_search = ros::Time::now();
        current_region = tree_->getLeafRegionContaining(current->pose->x(),current->pose->y());
        end_leaf_search = ros::Time::now();
        leaf_time_.push_back(ros::Duration(end_leaf_search-start_leaf_search).toSec());

        //Early exit
        if(current_region==tree_->getLeafRegionContaining(v_goal_->pose->x(),v_goal_->pose->y())){
            ROS_INFO_STREAM("Early exit");
            v_close_ = current;
            break;
        }

        if(!ros::ok()){
            std::cout << "Stopping Hybrid A* prematurely" << std::endl;
            saveDataContainers();
            exit(1);
        }
        

        double distance = getDistance(current->pose,v_goal_->pose);
        std::cout << "Distance: " << distance <<std::endl;
        double sim_time;
        if(enable_adaptive_sim_time_){
            sim_time = adaptiveSimulationTime(current,distance);
        } else{
            sim_time = default_sim_time_;
        }

        //Calculate vertex candidates
        std::vector<std::pair<extendedVertex*, double>> candidates_container_;
        for(auto heading_candidate_it=heading_candidates.begin();heading_candidate_it!=heading_candidates.end();heading_candidate_it++){
            //Calculate distance to region border
            double heading = *heading_candidate_it+current->pose->w();
            state_type candidate_state = {current->pose->x(),current->pose->y(),current->pose->w(),current->twist->x(),current->twist->y(),current->twist->z()};
            ModelLibrary::simulatedHorizon hor = simulateVessel(candidate_state,heading,sim_time);
            
            if (similarClosed(candidate_state)) continue;
            if (collision(candidate_state,current_region,hor)) continue;
            
            std::pair<extendedVertex*,bool> next_pair = getNextVertex(candidate_state);

            double new_cost = cost_so_far_[current] + getDistance(current->pose,next_pair.first->pose);
            if(!next_pair.second || new_cost<cost_so_far_[next_pair.first]){
                cost_so_far_[next_pair.first]=new_cost;
                double priority = heuristic(current,next_pair.first,new_cost);
                candidates_container_.push_back(std::make_pair(next_pair.first,priority-new_cost));
                frontier_.put(next_pair.first,priority);
                came_from_[next_pair.first]=current;
            }

        }
        candidate_exploration_.push_back(std::make_pair(current,candidates_container_));
    }
    path_ = reconstructPath();
    end_search_ = ros::Time::now();
    if (save_search_data_) saveDataContainers();
    if (save_benchmark_data_) dumpSearchBenchmark();
}

/**
 * @brief Self explanatory
 * 
 * @return int VertexID
 */
int HybridAStar::generateVertexID(){
    return vertex_id_++;
}

/**
 * @brief Get the true distance between two positions (lon,lat,0) using GeographicLib
 * 
 * @param u Position A
 * @param v Position B
 * @return double True distance (estimate) [m]
 */
double HybridAStar::getDistance(StateVec* u, StateVec* v){
    ros::Time start = ros::Time::now();
    double distance;
    geod_.Inverse(u->y(),u->x(),v->y(),v->x(),distance);
    get_distance_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return abs(distance);
}


/**
 * @brief Get a coarse collision-free-path length approximation using A* on Quadtree graph
 * 
 * @param u Position A 
 * @param v Position B
 * @return double DIstance approximation [m]
 */
double HybridAStar::getGridDistance(StateVec* u, StateVec* v){
    //Check if roughly this start->goal config has been checked before
    Region* current = tree_->getLeafRegionContaining(u->x(),u->y());
    auto grid_lookup_it = grid_distance_lookup_.find(current);

    double distance_u_v = 0;
    double distance_centroid_v = 0;
    geod_.Inverse(u->y(),u->x(),v->y(),v->x(),distance_u_v);
    geod_.Inverse(current->centroid_.getY(),current->centroid_.getX(),v->y(),v->x(),distance_centroid_v);
    double difference = abs(distance_u_v)-abs(distance_centroid_v);
    
    if (grid_lookup_it!=grid_distance_lookup_.end()){
        return (*grid_lookup_it).second+difference;
    }

    tree_->setStart(current->centroid_.getX(),current->centroid_.getY());
    tree_->setGoal(v->x(),v->y());

    grid_search_alg_->setStart(current->centroid_.getX(),current->centroid_.getY());
    grid_search_alg_->setGoal(v->x(),v->y());
    bool search_successful = grid_search_alg_->search();
    if(!search_successful){
        //Due to fundamental bug in quadtree builder, the search will in very rare cases be unsuccessful.
        return -1;
    }
    std::vector<Vertex*> shortest_path = grid_search_alg_->getPath();
    double spline_distance=0;
    double total_distance=0;
    for(int i=0; i<shortest_path.size()-1; i++){
        geod_.Inverse(shortest_path[i]->state.y(),shortest_path[i]->state.x(),shortest_path[i+1]->state.y(),shortest_path[i+1]->state.x(),spline_distance);
        total_distance+=abs(spline_distance);
    }
    grid_distance_lookup_.insert(std::make_pair(current,total_distance));
    return total_distance+difference;
}

/**
 * @brief Get a coarse collision-free-path length approximation using A* on Quadtree graph
 * 
 * @param u Position A 
 * @param v Position B
 * @return double DIstance approximation [m]
 */
double HybridAStar::getGridDistanceAccurate(StateVec* u, StateVec* v){
    //Check if roughly this start->goal config has been checked before

    tree_->setStart(u->x(),u->y());
    tree_->setGoal(v->x(),v->y());

    grid_search_alg_->setStart(u->x(),u->y());
    grid_search_alg_->setGoal(v->x(),v->y());
    bool search_successful = grid_search_alg_->search();
    if(!search_successful){
        //Due to fundamental bug in quadtree builder, the search will in very rare cases be unsuccessful.
        return -1;
    }
    std::vector<Vertex*> shortest_path = grid_search_alg_->getPath();
    double spline_distance=0;
    double total_distance=0;
    for(int i=0; i<shortest_path.size()-1; i++){
        geod_.Inverse(shortest_path[i]->state.y(),shortest_path[i]->state.x(),shortest_path[i+1]->state.y(),shortest_path[i+1]->state.x(),spline_distance);
        total_distance+=abs(spline_distance);
    }
    return total_distance;
}

/**
 * @brief Given a vessel state, return search vertex and boolean flag informing if has been previously explored.
 * 
 * @remark Currently, heading is not taken into consideration when comparing internally in this function.
 * 
 * @param next_state The vessel state
 * @return std::pair<extendedVertex*,bool>
 */
std::pair<extendedVertex*,bool> HybridAStar::getNextVertex(state_type& next_state){
    //Find most similar node prdviously explored (if exists)
    ros::Time start = ros::Time::now();
    double distance_check;
    bool explored = false;
    for(auto vertex_it=cost_so_far_.begin(); vertex_it!=cost_so_far_.end();vertex_it++){
        geod_.Inverse((*vertex_it).first->pose->y(),(*vertex_it).first->pose->x(),next_state[1],next_state[0],distance_check);
        if (abs(distance_check)<prune_radius_explored_){
            explored=true;
            get_next_vertex_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
            return std::make_pair((*vertex_it).first,explored);
        }
    }
    get_next_vertex_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return std::make_pair(new extendedVertex(generateVertexID(),next_state),explored);
}

/**
 * @brief Check if simulation horizon collides with land. 
 * 
 * @details Includes Two layers of fast-checks before consulting the GDAL API, both utilizing region comparison.
 * 
 * @param current_state Current state
 * @param current_region Current region
 * @param sim_hor Candidate simulation horizon
 * @return true Collision with land detected
 * @return false Collision with land not detected
 */
bool HybridAStar::collision(state_type& current_state, Region* current_region, ModelLibrary::simulatedHorizon& sim_hor){
    //If in same region as last time, cant possibly be collision
    ros::Time start_collision = ros::Time::now();
    Region* candidate_region = tree_->getLeafRegionContaining(current_state[0],current_state[1]);

    if (candidate_region==current_region){
        collision_time_.push_back(ros::Duration(ros::Time::now()-start_collision).toSec());
        return false;
    }

    //If candidate region is nullptr, definitely path into land, no need to check for detailed collision
    if(candidate_region==nullptr){
        points_outside_quadtree_.push_back(std::make_pair(current_state[0],current_state[1]));
        collision_time_.push_back(ros::Duration(ros::Time::now()-start_collision).toSec());
        return true;
    }

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
    
    if (map_service_->intersects(&spline_,LayerID::COLLISION)){
        collision_time_.push_back(ros::Duration(ros::Time::now()-start_collision).toSec());
        return true;
    } else{
        collision_time_.push_back(ros::Duration(ros::Time::now()-start_collision).toSec());
        return false;
    }
}

double HybridAStar::breakTie(StateVec* current){
    double dx1, dy1, dx2, dy2;
    return 0;
}

/**
 * @brief The Hybrid A* Heuristic specially developed and tuned for this application.
 * 
 * @param current The current state
 * @param next The candidate state 
 * @param new_cost The cost-so-far, including distance cost of moving from current->next
 * @param search_phase The current search phase
 * @return double The heuristic value
 */
double HybridAStar::heuristic(extendedVertex* current,extendedVertex* next, double new_cost){
    ros::Time start_heuristic = ros::Time::now();
    double voronoi_field = 1e5*map_service_->voronoi_field(next->pose->x(),next->pose->y());
    double cost_distance = voronoi_field_cost_weight_*voronoi_field;

    double priority = new_cost + cost_distance;
    priority+=distance_scaling_factor_*std::max(getDistance(next->pose,v_goal_->pose),getGridDistanceAccurate(next->pose,v_goal_->pose));
    heuristic_time_.push_back(ros::Duration(ros::Time::now()-start_heuristic).toSec());
    return priority;
}

/**
 * @brief When a search is complete, reconstructs the path from start to goal using the came_from_ map.
 * 
 * @return std::vector<extendedVertex*> Path
 */
std::vector<extendedVertex*> HybridAStar::reconstructPath() {
    ros::Time start = ros::Time::now();
    std::vector<extendedVertex*> path;
    if(v_goal_!=v_close_){
        path.push_back(v_goal_);
    }
    extendedVertex* current = v_close_;
    while (current != v_start_) {
        path.push_back(current);
        current = came_from_[current];
    }
    path.push_back(v_start_);
    std::reverse(path.begin(), path.end());
    reconstruct_path_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return path;
}

/**
 * @brief Get the currently stored path
 * 
 * @return std::vector<extendedVertex*> Path
 */
std::vector<extendedVertex*> HybridAStar::getPath(){
    return path_;
}

double HybridAStar::adaptiveSimulationTime(extendedVertex* current,double distance_to_goal){
    ros::Time start_calc_sim = ros::Time::now();
    double dist_land = 1e5*map_service_->distance(current->pose->x(), current->pose->y(),LayerID::COLLISION,INFINITY);
    double sim_time = std::min(std::max(dist_land/5.0,underway_sim_time_minimum_),std::max(approach_sim_time_scaling_*distance_to_goal/5.0,approach_sim_time_minimum_));
    ros::Time end_calc_sim = ros::Time::now();
    calc_sim_time_.push_back(ros::Duration(end_calc_sim-start_calc_sim).toSec());
    return sim_time;
}

/**
 * @brief Simulate the vessel based on a heading candidate.
 * 
 * @param state Current vessel state
 * @param heading_candidate Heading candidate (absolute) [rad]
 * @param sim_time Simulation time [s]
 * @return ModelLibrary::simulatedHorizon 
 */
ModelLibrary::simulatedHorizon HybridAStar::simulateVessel(state_type& state, double heading_candidate, double sim_time){
    ros::Time start_sim = ros::Time::now();
    Eigen::Vector3d pos_wgs(state[0],state[1],0);
    Eigen::Vector3d pos_enu;
    geo_converter_.convert("WGS84",pos_wgs,"start_enu",&pos_enu);
    state_type candidate_state = {pos_enu.x(),pos_enu.y(),state[2],state[3],state[4],state[5]};
    ModelLibrary::simulatedHorizon sim_hor = vessel_model_->simulateHorizonAdaptive(candidate_state,3,heading_candidate,sim_time);

    Eigen::Vector3d candidate_enu(candidate_state[0],candidate_state[1],0);
    Eigen::Vector3d candidate_wgs;
    geo_converter_.convert("start_enu",candidate_enu,"WGS84",&candidate_wgs);

    state = {candidate_wgs.x(),candidate_wgs.y(),candidate_state[2],candidate_state[3],candidate_state[4],candidate_state[5]};
    ros::Time end_sim = ros::Time::now();
    simulate_time_.push_back(ros::Duration(end_sim-start_sim).toSec());
    return sim_hor;
}

/**
 * @brief Evaluate the state has already previously been assigned with a search vertex that has been closed.
 * 
 * @remark Does not take heading into account in comparison checking
 * 
 * @param state 
 * @return true 
 * @return false 
 */
bool HybridAStar::similarClosed(state_type& state){
    //If vertex sufficiently simiar has been closed, go to next candidate
    ros::Time start = ros::Time::now();
    double closed_distance_check;
    bool next_closed =false;
    for(auto closed_it = closed_.begin(); closed_it!=closed_.end(); closed_it++){
        geod_.Inverse((*closed_it)->pose->y(),(*closed_it)->pose->x(),state[1],state[0],closed_distance_check);
        if(abs(closed_distance_check)<prune_radius_closed_){
            similar_closed_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
            return true;
        }
    }
    similar_closed_time_.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return false;
}

/**
 * @brief Save data containers for debugging/visualization purposes
 * 
 */
void HybridAStar::saveDataContainers(){
    std::cout << "Saving data containers" << std::endl;
    std::string path = ros::package::getPath("usv_mission_planner")+"/data/missions/"+mission_name_+"/hybrid_astar/";
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directories(path);
    }

    std::ofstream closed_file(path+"closed.csv");
    closed_file << "lon,lat,psi,u,v,r,id\n";

    std::ofstream came_from_file(path+"came_from.csv");
    came_from_file << "lon_from,lat_from,psi_from,lon_to,lat_to,psi_to\n";

    std::ofstream frontier_file(path+"frontier.csv");
    frontier_file << "lon,lat,psi,u,v,r,id\n";

    std::ofstream path_file(path+"path.csv");
    path_file << "lon,lat\n";

    std::ofstream points_outside_quadtree_file(path + "outside_quadtree.csv");
    points_outside_quadtree_file << "lon,lat\n";

    std::ofstream candidate_exploration_file(path + "candidate_exploration.csv");
    candidate_exploration_file << "origin_lon,origin_lat,cand_lon,cand_lat,cand_h"<<"\n";

    for(auto path_it=path_.begin(); path_it!=path_.end(); path_it++){
        path_file << (*path_it)->pose->x() << "," << (*path_it)->pose->y() << std::endl;
    }
    for (auto came_from_it = came_from_.begin(); came_from_it!=came_from_.end(); came_from_it++){
        extendedVertex* fr = (*came_from_it).second;
        extendedVertex* to = (*came_from_it).first;
        came_from_file<<fr->pose->x()<<","<<fr->pose->y()<<","<<fr->pose->w()<<","<<to->pose->x()<<","<<to->pose->y()<<","<<to->pose->w()<<"\n";
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

    for(auto it = candidate_exploration_.begin(); it!= candidate_exploration_.end(); it++){
        for(auto candidate_it=(*it).second.begin(); candidate_it!=(*it).second.end(); candidate_it++){
            candidate_exploration_file<<(*it).first->pose->x()<<","<<(*it).first->pose->y()<<","<<(*candidate_it).first->pose->x()<<","<<(*candidate_it).first->pose->y()<<","<<(*candidate_it).second<<"\n";
        }
    }

    closed_file.close();
    came_from_file.close();
    frontier_file.close();
    path_file.close();
    points_outside_quadtree_file.close();
    candidate_exploration_file.close();
}

/**
 * @brief Dump benchmark data to console and csv files for debugging and performance evaluation.
 * 
 */
void HybridAStar::dumpSearchBenchmark(){
    std::string path = ros::package::getPath("usv_mission_planner")+"/data/missions/"+mission_name_+"/hybrid_astar/";
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directories(path);
    }

    std::ofstream benchmark_file_time(path+"benchmark_time.csv");
    std::ofstream benchmark_file_misc(path+"benchmark_misc.csv");
    benchmark_file_time<<"name,times_run,total_time\n";
    benchmark_file_misc<<"name,value\n";

    //Benchmark info:
    double total_distance = 0;
    double spline_distance = 0;
    for (int i=0; i!=path_.size()-1; i++){
        geod_.Inverse(path_[i]->pose->y(),path_[i]->pose->x(),path_[i+1]->pose->y(),path_[i+1]->pose->x(),spline_distance);
        total_distance+=spline_distance;
    }
    ROS_INFO_STREAM("Total distance to travel: " << total_distance << " m" );
    benchmark_file_misc<<"total_distance"<<","<<total_distance<<"\n";

    double min_distance_to_land = INFINITY;
    double accumulated_distance_to_land = 0.0;
    for(auto path_it = path_.begin(); path_it!=path_.end(); path_it++){
        double distance_to_land = map_service_->distance((*path_it)->pose->x(),(*path_it)->pose->y(),LayerID::COLLISION);
        if(distance_to_land<min_distance_to_land){
            min_distance_to_land = distance_to_land;
        }
        accumulated_distance_to_land+=distance_to_land;
    }
    ROS_INFO_STREAM("Min. distance to land: " << min_distance_to_land*1e5);
    ROS_INFO_STREAM("Accumulated distance to land: " << accumulated_distance_to_land*1e5);
    benchmark_file_misc<<"min_distance_land"<<","<<min_distance_to_land*1e5<<"\n";
    benchmark_file_misc<<"accumulated_distance_land"<<","<<accumulated_distance_to_land*1e5<<"\n";


    ROS_INFO_STREAM("Search took: " << ros::Duration(end_search_-start_search_).toSec());
    ROS_INFO_STREAM("Check for leaf " << leaf_time_.size() << " times. Total time spent: " << std::accumulate(leaf_time_.begin(),leaf_time_.end(),0.0));
    ROS_INFO_STREAM("Check for collision " << collision_time_.size() << " times. Total time spent: " << std::accumulate(collision_time_.begin(),collision_time_.end(),0.0));
    ROS_INFO_STREAM("Simulate " << simulate_time_.size() << " times. Total time spent: " << std::accumulate(simulate_time_.begin(),simulate_time_.end(),0.0));
    ROS_INFO_STREAM("Calculate heuristic " << heuristic_time_.size() << " times. Total time spent: " << std::accumulate(heuristic_time_.begin(),heuristic_time_.end(),0.0));
    ROS_INFO_STREAM("Calculate sim time " << calc_sim_time_.size() << " times. Total time spent: " << std::accumulate(calc_sim_time_.begin(),calc_sim_time_.end(),0.0));
    benchmark_file_time<<"search_time"<<","<<1<<","<<ros::Duration(end_search_-start_search_).toSec()<<"\n";
    benchmark_file_time<<"check_for_leaf"<<","<<leaf_time_.size()<<","<<std::accumulate(leaf_time_.begin(),leaf_time_.end(),0.0)<<"\n";
    benchmark_file_time<<"check_for_collision"<<","<<collision_time_.size()<<","<<std::accumulate(collision_time_.begin(),collision_time_.end(),0.0)<<"\n";
    benchmark_file_time<<"simulate"<<","<<simulate_time_.size()<<","<<std::accumulate(simulate_time_.begin(),simulate_time_.end(),0.0)<<"\n";
    benchmark_file_time<<"calculate_heuristic"<<","<<heuristic_time_.size()<<","<<std::accumulate(heuristic_time_.begin(),heuristic_time_.end(),0.0)<<"\n";
    benchmark_file_time<<"calculate_sim_time"<<","<<calc_sim_time_.size()<<","<<std::accumulate(calc_sim_time_.begin(),calc_sim_time_.end(),0.0)<<"\n";
    benchmark_file_time<<"similar_closed"<<","<<similar_closed_time_.size()<<","<<std::accumulate(similar_closed_time_.begin(), similar_closed_time_.end(),0.0)<<"\n";
    benchmark_file_time<<"get_next_vertex"<<","<<get_next_vertex_time_.size()<<","<<std::accumulate(get_next_vertex_time_.begin(), get_next_vertex_time_.end(), 0.0)<<"\n";
    benchmark_file_time<<"get_distance"<<","<<get_distance_time_.size()<<","<<std::accumulate(get_distance_time_.begin(), get_distance_time_.end(), 0.0)<<"\n";
    benchmark_file_time<<"reconstruct_path"<<","<<reconstruct_path_time_.size()<<","<<std::accumulate(reconstruct_path_time_.begin(), reconstruct_path_time_.end(), 0.0)<<"\n";
    ROS_INFO_STREAM("Grid search algorithm:");
    grid_search_alg_->dumpSearchBenchmark();
}

HybridAStarROS::HybridAStarROS(ros::NodeHandle& nh, Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapService* map_service, std::string mission_name):
nh_(nh),
HybridAStar(tree,vessel_model,map_service,mission_name){
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