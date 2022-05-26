#pragma once

#include "usv_model/model_library.h"
#include "usv_map/quadtree.h"
#include "GeographicLib/Geodesic.hpp"
#include "geotf/geodetic_converter.h"
#include "vector"
#include "usv_map/map_service.h"
#include "unordered_map"
#include <iostream>
#include "usv_mission_planner/priority_queue.h"
#include "usv_mission_planner/astar.h"

struct extendedVertex{
    extendedVertex(int id,state_type& state){
        id_ = id;
        pose = new StateVec(state[0],state[1],0,state[2]);
        twist = new Eigen::Vector3d(state[3],state[4],state[5]);
    }
    int id_;
    StateVec* pose;
    Eigen::Vector3d* twist;
    ~extendedVertex(){
        delete pose;
        delete twist;    
    }
};

struct procedureBenchmark{
    int calls_;
    double accumulated_time_;
    procedureBenchmark(int calls, double accumulated_time):calls_(calls),accumulated_time_(accumulated_time){}
};

class HybridAStar{
    public:
        HybridAStar(Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapService* map_service, std::string mission_name);
        void setStart(double lon, double lat, double yaw);
        void setGoal(double lon, double lat, double yaw);
        void setMissionName(std::string mission_name);
        void search();
        std::vector<extendedVertex*> getPath();
    
    protected:
        geotf::GeodeticConverter geo_converter_;
        GeographicLib::Geodesic geod_;
        MapService* map_service_;

        Quadtree* tree_;
        AStar* grid_search_alg_;
        ModelLibrary::Viknes830* vessel_model_;

        extendedVertex* v_start_;
        extendedVertex* v_goal_;
        extendedVertex* v_close_;

        std::vector<extendedVertex*> path_;
        OGRLineString spline_;

        std::unordered_map<extendedVertex*, extendedVertex*> came_from_;
        std::unordered_map<extendedVertex*, double> cost_so_far_;
        std::vector<extendedVertex*> closed_; 
        PriorityQueue<extendedVertex*,double> frontier_;

        std::unordered_map<Region*, double> grid_distance_lookup_;

        //Search pruning
        double prune_radius_explored_;
        double prune_radius_closed_;
        //Static simulation time
        double default_sim_time_;
        //Adaptive simulation time
        bool enable_adaptive_sim_time_;
        double underway_sim_time_minimum_;
        double approach_sim_time_scaling_;
        double approach_sim_time_minimum_;
        //Heuristic
        double voronoi_field_cost_weight_;
        double distance_scaling_factor_;
        //Data storage
        bool save_search_data_;
        bool save_benchmark_data_;

        int vertex_id_=0;
        int generateVertexID();
        void clear();
        std::vector<extendedVertex*> reconstructPath();

        double getDistance(StateVec* u, StateVec* v);
        double getGridDistance(StateVec* u, StateVec* v);
        double getGridDistanceAccurate(StateVec* u, StateVec* v);
        std::pair<extendedVertex*,bool> getNextVertex(state_type& next_state);
        bool collision(state_type& current_state, Region* current_region, ModelLibrary::simulatedHorizon& sim_hor);
        bool tssLane(extendedVertex* current);
        bool tssViolation(extendedVertex* current,state_type& candidate, double heading);
        double breakTie(StateVec* current);
        double heuristic(extendedVertex* current,extendedVertex* next,double heading,double new_cost);

        double adaptiveSimulationTime(extendedVertex* current, double distance_to_goal);
        ModelLibrary::simulatedHorizon simulateVessel(state_type& state, double heading_candidate, double sim_time);
        bool similarClosed(state_type& state);

        //Debug tools
        std::string mission_name_;
        std::vector<std::pair<extendedVertex*,std::vector<std::pair<extendedVertex*,double>>>> candidate_exploration_;
        std::vector<std::pair<double,double>> points_outside_quadtree_;
        void saveDataContainers();

        //Benchmark tools
        ros::Time start_search_;
        ros::Time end_search_;
        std::vector<double> collision_time_accumulation_;
        std::vector<double> leaf_time_accumulation_;
        std::vector<double> simulate_time_accumulation_;
        std::vector<double> heuristic_time_accumulation_;
        std::vector<double> calcSimTime_time_accumulation_;
        std::vector<double> similar_closed_time_accumulation_;
        std::vector<double> get_next_vertex_time_accumulation_;
        std::vector<double> get_distance_time_accumulation_;
        std::vector<double> get_grid_distance_time_accumulation_;


        std::vector<std::pair<double,double>> candidateExploration_time;
        std::vector<std::pair<double,procedureBenchmark>> collision_time_;
        std::vector<std::pair<double,procedureBenchmark>> leaf_time_;
        std::vector<std::pair<double,procedureBenchmark>> simulate_time_;
        std::vector<std::pair<double,procedureBenchmark>> heuristic_time_;
        std::vector<std::pair<double,procedureBenchmark>> calcSimTime_time_;
        std::vector<std::pair<double,procedureBenchmark>> similar_closed_time_;
        std::vector<std::pair<double,procedureBenchmark>> get_next_vertex_time_;
        std::vector<std::pair<double,procedureBenchmark>> get_distance_time_;
        std::vector<std::pair<double,procedureBenchmark>> get_grid_distance_time_;
        std::vector<double> reconstruct_path_time_;

        std::vector<std::pair<double,double>> dist_to_land_vec_;
        std::vector<std::pair<double,double>> dist_to_goal_vec_;
        std::vector<std::pair<double,double>> sim_time_vec_;
        void clearAccumulationContainers();
        double getRelativeTime();
        void writeBenchmarkContainer(std::vector<std::pair<double,double>>& container,std::ofstream& outfile);
        void writeBenchmarkContainer(std::vector<std::pair<double,procedureBenchmark>>& container,std::ofstream& outfile);
        void dumpSearchBenchmark();

};

class HybridAStarROS : public HybridAStar{
    public:
        HybridAStarROS(ros::NodeHandle& nh,Quadtree* tree, ModelLibrary::Viknes830* vessel_model,MapService* map_service, std::string mission_name);
        void visualize();
    private:
        geotf::GeodeticConverter geo_converter_;
        ros::NodeHandle nh_;

        ros::Publisher path_marker_pub_;
        visualization_msgs::Marker path_marker_;

        void initializeMarkers();
        void addVisualPath();
        void publishVisualPath();
};