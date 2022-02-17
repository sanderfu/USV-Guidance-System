#pragma once

#include "ros/ros.h"
#include "ros/package.h"
#include "gdal/ogrsf_frmts.h"
#include "planner_common/graph_manager.h"
#include "vector"
#include "numeric"
#include "queue"
#include <unordered_map>
#include <omp.h>


#include "visualization_msgs/Marker.h"
#include "geotf/geodetic_converter.h"
#include <iostream>
#include <fstream>

#include "GeographicLib/Geodesic.hpp"

enum regionEdge{
    N,S,E,W
};

enum childRegion{
    NW, NE, SW, SE
};

typedef struct {
    //Overall info
    double build_time;

    std::vector<double> splitRegion_time;
    std::vector<double> getOccupiedArea_time;

    //
    //ros::Time get_frame_points_start;
    //ros::Time get_frame_points_end;
    //float get_frame_points_total_time;
} quadtree_benchmark_t;

class Region{
    public:
        Region(OGRPoint lower_left, OGRPoint upper_right, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds);
        Region(double lon_lower, double lat_lower, double lon_upper, double lat_upper, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds);
        Region* getChildRegionContaining(double lon, double lat);
        void addChild(Region* child_region_ptr, childRegion child_region);

        double getWidth();
        double getHeight();
        double getOccupiedRatio();
        double getOccupiedArea();
        double getArea();
        int getDepth();
        int getID();
        int getParentID();
        childRegion getOwnRegion();
        Region* getChildRegion(childRegion region_position);

        OGRPoint lower_left_;
        OGRPoint upper_right_;
        OGRPoint centroid_;

        OGRPolygon* region_polygon_;

        std::unordered_map<childRegion, Region*> children;
        std::vector<Vertex*> vertices;
    private:
        GDALDataset* ds_;
        OGRLayer* comparison_layer_;

        int depth_;
        int id_;
        int parent_id_;
        childRegion own_region_;
};

class Quadtree{
    public:
        Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, bool build_immediately=true);
        void setStart(Vertex* s);
        void setGoal(Vertex* g);

        Region* getLeafRegionContaining(double lon, double lat);

        void save(const std::string& tree_name);
        void load(const std::string& tree_name);

        //Debug
        std::vector<Region*> region_sequence_;

    protected:
        GDALDataset* ds_;
        GraphManager* gm_;

        Region* tree_root_;

        OGRPoint lower_left_;
        OGRPoint upper_right_;

        int region_id_=0;

        GeographicLib::Geodesic geod_;

        //Benchmark
        quadtree_benchmark_t benchmark_data_;

        std::unordered_map<regionEdge,std::vector<StateVec>> getFramePoints(Region* region);
        void build();

        int generateRegionID();
        void splitRegion(Region* region, std::queue<Region*>& regions_to_evaluate);
        Region* findLeafRegionContaining(StateVec& pos);
        void setCustomVertex(Vertex* s);
};

class QuadtreeROS : public Quadtree{
    public:
        QuadtreeROS(ros::NodeHandle& nh, OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, bool build_immediately=true);
        void visualize();
        void testGetRegion(double lon, double lat);
    private:
        geotf::GeodeticConverter geo_converter_;
        ros::NodeHandle nh_;

        ros::Publisher vertex_marker_pub_;
        ros::Publisher edge_marker_pub_;
        ros::Publisher region_marker_pub_;
        ros::Publisher test_point_pub_;

        visualization_msgs::Marker region_marker_;
        visualization_msgs::Marker test_point_;
        visualization_msgs::Marker edge_marker_;
        visualization_msgs::Marker vertex_marker_;

        //Visualization
        void highlightRegion(Region* region);

        void initializeMarkers();
        void addVisualVertex(Eigen::Vector3d& vertex);
        void addVisualEdge(Eigen::Vector3d& from_vertex, Eigen::Vector3d& to_vertex);
        
        inline void publishVisualGraph(){
            std::cout << "Edge marker frame id:" << edge_marker_.header.frame_id << std::endl;
            vertex_marker_pub_.publish(vertex_marker_);
            edge_marker_pub_.publish(edge_marker_);
        }
};
