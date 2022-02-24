#pragma once

#include "usv_model/model_library.h"
#include "usv_mission_planner/quadtree.h"
#include "GeographicLib/Geodesic.hpp"
#include "geotf/geodetic_converter.h"
#include "vector"
#include "usv_map/map_service.h"
#include "unordered_map"
#include <iostream>
#include "usv_mission_planner/flexible_priority_queue.h"
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

class HybridAStar{
    public:
        HybridAStar(Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapServiceClient* map_client);
        void setStart(double lon, double lat, double yaw);
        void setGoal(double lon, double lat, double yaw);
        void search();

        std::vector<extendedVertex*> getPath();
    
    protected:
        geotf::GeodeticConverter geo_converter_;
        GeographicLib::Geodesic geod_;
        MapServiceClient* map_client_;

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

        int vertex_id_=0;
        int generateVertexID();

        double getDistance(StateVec* u, StateVec* v);
        double getGridDistance(StateVec* u, StateVec* v);
        bool collision(ModelLibrary::simulatedHorizon& sim_hor);
        std::vector<extendedVertex*> reconstructPath();

        double SSA(double angle){
            return fmod(angle+M_PI,2*M_PI) - M_PI;
        }

        //Debug tools
        std::vector<std::pair<double,double>> points_outside_quadtree_;
        void saveDataContainers();
        void dumpSearchBenchmark();

};

class HybridAStarROS : public HybridAStar{
    public:
        HybridAStarROS(ros::NodeHandle& nh,Quadtree* tree, ModelLibrary::Viknes830* vessel_model, MapServiceClient* map_client);
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