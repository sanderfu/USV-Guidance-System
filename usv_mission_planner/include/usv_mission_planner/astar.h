#pragma once

#include "planner_common/graph_manager.h"
#include "usv_mission_planner/priority_queue.h"
#include "GeographicLib/Geodesic.hpp"
#include "geotf/geodetic_converter.h"

#include "visualization_msgs/Marker.h"

class AStar{
    public:
        AStar(GraphManager* gm);
        void setStart(double lon, double lat);
        void setGoal(double lon, double lat);
        bool search();

        //For visualization/debugging
        std::vector<Vertex*> getPath();
    protected:
        std::vector<Vertex*> path_;
        GraphManager* gm_;
        GeographicLib::Geodesic geod_;
        Vertex* v_start_;
        Vertex* v_goal_;

        double heuristic(const StateVec& state_u, const StateVec& state_v);
        std::vector<Vertex*> reconstructPath(std::unordered_map<Vertex*, Vertex*>& came_from);
};

class AStarROS : public AStar{
    public:
        AStarROS(ros::NodeHandle& nh, GraphManager* gm);
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