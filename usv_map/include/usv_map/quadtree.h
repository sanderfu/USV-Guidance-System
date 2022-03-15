#pragma once

//Dependencies for quadtree
#include "ros/ros.h"
#include "ros/package.h"
#include "usv_map/region.h"
#include "gdal/ogrsf_frmts.h"
#include "planner_common/graph_manager.h"
#include "vector"
#include "queue"
#include "unordered_map"
#include "omp.h"
#include "GeographicLib/Geodesic.hpp"
#include "geotf/geodetic_converter.h"
#include "iostream"
#include "fstream"

//Dependencies for quadtree RVIZ visualization
#include "visualization_msgs/Marker.h"

/**
 * @brief Container for all benchmarking data
 * 
 */
typedef struct {
    //Overall info
    double build_time;

    std::vector<double> splitRegion_time;
    std::vector<double> getOccupiedArea_time;

} quadtree_benchmark_t;

/**
 * @brief Class for building, using, loading and saving regional (framed)
 * quadtree.
 * 
 */
class Quadtree{
    public:
        Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, std::string mission_region, bool build_immediately=true);
        
        void setStart(double lon, double lat);
        void setGoal(double lon, double lat);
        Region* getLeafRegionContaining(double lon, double lat);
        GraphManager* getGraphManager();

        void save(const std::string& mission_region);
        void load(const std::string& Mission_region);

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
        void setCustomVertex(Vertex* s);

        //Debug
        std::vector<Region*> region_sequence_;
};

class QuadtreeROS : public Quadtree{
    public:
        QuadtreeROS(ros::NodeHandle& nh, OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, std::string mission_region, bool build_immediately=true);
        void visualize();
        Region* testGetRegion(double lon, double lat);
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
