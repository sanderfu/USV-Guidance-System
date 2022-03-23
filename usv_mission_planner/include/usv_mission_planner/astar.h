#pragma once

#include "planner_common/graph_manager.h"
#include "usv_mission_planner/priority_queue.h"
#include "GeographicLib/Geodesic.hpp"
#include "geotf/geodetic_converter.h"
#include "usv_map/map_service.h"

#include "visualization_msgs/Marker.h"

class AStar{
    public:
        AStar(GraphManager* gm,MapService* map_service, std::string mission_name);
        void setStart(double lon, double lat);
        void setGoal(double lon, double lat);
        bool search();

        //For visualization/debugging
        std::vector<Vertex*> getPath();
        void saveDataContainers(int search_id);
    protected:
    std::string mission_name_;
        std::vector<Vertex*> path_;
        double path_length_;
        GraphManager* gm_;
        GeographicLib::Geodesic geod_;
        Vertex* v_start_;
        Vertex* v_goal_;
        Vertex* v_attachement_;
        MapService* map_service_;
        std::unordered_map<Vertex*, Vertex*> came_from_;
        std::unordered_map<Vertex*, double> cost_so_far_;
        PriorityQueue<Vertex*,double> frontier_;
        std::unordered_set<int> closed_;

        std::unordered_map<int, int> path_lookup_table_;
        std::unordered_map<int,double> distance_lookup_table_;
        
        int search_id_;
        int generateSearchID();
        double heuristicDirect(const StateVec& state_u, const StateVec& state_v);
        bool reconstructPath();
        bool vertexInLookupTable(const Vertex* v);
        void updateLookupTable();
        bool reconstructPathFromLookup(Vertex* v);
        bool followingStoredPath(Vertex* v);
};

class AStarROS : public AStar{
    public:
        AStarROS(ros::NodeHandle& nh, GraphManager* gm,MapService* map_service);
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