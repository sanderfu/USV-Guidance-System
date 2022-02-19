#pragma once

#include "usv_model/model_library.h"
#include "usv_mission_planner/quadtree.h"
#include "GeographicLib/Geodesic.hpp"
#include "geotf/geodetic_converter.h"
#include "usv_mission_planner/priority_queue.h"
#include "vector"
#include "usv_map/map_service.h"
#include "unordered_map"

struct extendedVertex{
    extendedVertex(int id,state_type& state){
        vertex = new Vertex(id,StateVec(state[0],state[1],0,state[2]));
        twist = new Eigen::Vector3d(state[3],state[4],state[5]);
    }
    Vertex* vertex;
    Eigen::Vector3d* twist;
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
        ModelLibrary::Viknes830* vessel_model_;

        extendedVertex* v_start_;
        extendedVertex* v_goal_;
        extendedVertex* v_close_;

        std::vector<extendedVertex*> path_;

        int vertex_id_=0;
        int generateVertexID();

        double getDistance(Vertex* u, Vertex* v);
        bool collision(ModelLibrary::simulatedHorizon& sim_hor);
        std::vector<extendedVertex*> reconstructPath(std::unordered_map<extendedVertex*, extendedVertex*>& came_from);

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