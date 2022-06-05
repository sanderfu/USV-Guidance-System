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
#include "usv_map/map_service.h"

//Dependencies for quadtree RVIZ visualization
#include "visualization_msgs/Marker.h"

/**
 * @brief Container for all benchmarking data
 * 
 */
typedef struct {
    //Overall info
    double build_time;
    int vertices;

    std::vector<double> splitRegion_time;
    std::vector<double> getOccupiedArea_time;
    std::vector<double> getFramePoints_time;

} quadtree_benchmark_t;

/**
 * @brief Class for building, using, loading and saving regional (framed)
 * quadtree.
 * 
 */
class Quadtree{
    public:
        Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, GDALDataset* ds_detailed, std::string mission_region, MapService* map_service,bool build_immediately=true);
        
        void setStart(double lon, double lat);
        void setGoal(double lon, double lat);
        Region* getLeafRegionContaining(double lon, double lat);
        GraphManager* getGraphManager();

        void save(const std::string& mission_region);
        void saveForVisualization(const std::string& quadtree_name);
        void load(const std::string& Mission_region);
        void dumpBenchmark();

    protected:
        GDALDataset* ds_;
        GDALDataset* ds_detailed_;
        GraphManager* gm_;

        Region* tree_root_;

        OGRPoint lower_left_;
        OGRPoint upper_right_;

        int region_id_=0;

        GeographicLib::Geodesic geod_;
        MapService* map_service_;

        //Benchmark
        quadtree_benchmark_t benchmark_data_;

        std::unordered_map<regionEdge,std::vector<StateVec>> getFramePoints(Region* region);
        void build();

        int generateRegionID();
        void splitRegion(Region* region, std::queue<Region*>& regions_to_evaluate);
        void setCustomVertex(Vertex* s);
        std::string mission_region_;

        //Parameters
        bool fixed_divisor_flag_;
        int fixed_divisor_value_;
        double max_length_divisor_value_;

        //Debug
        std::vector<Region*> region_sequence_;
};
