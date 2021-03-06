#pragma once

#include "ros/ros.h"
#include "gdal/ogrsf_frmts.h"
#include "planner_common/graph_base.h"
#include "vector"
#include "unordered_map"
#include "omp.h"
#include "usv_map/map_service.h"
#include <random>

/**
 * @brief Enum describing the four edges of a region.
 * 
 */
enum regionEdge{
    N,S,E,W
};

/**
 * @brief Enum describing the four child regions of a parent region.
 * 
 */
enum childRegion{
    NW, NE, SW, SE
};

/**
 * @brief A class for regions, the elements of which a regional quadtree is built.
 * 
 */
class Region{
    public:
        Region(OGRPoint lower_left, OGRPoint upper_right, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds, GDALDataset* ds_detailed, MapService* map_service);
        Region(double lon_lower, double lat_lower, double width, double height, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds, GDALDataset* ds_detailed, MapService* map_service);
        
        int getID();
        int getDepth();
        double getArea();
        int getParentID();
        double getWidth();
        double getHeight();
        double getOccupiedRatio();
        double getOccupiedArea();
        childRegion getOwnRegion();
        Region* getChildRegion(childRegion region_position);
        Region* getChildRegionContaining(double lon, double lat);

        void addChild(Region* child_region_ptr, childRegion child_region);

        OGRPoint lower_left_;
        OGRPoint upper_right_;
        OGRPoint centroid_;

        OGRPolygon* region_polygon_;

        std::unordered_map<childRegion, Region*> children;
        std::vector<Vertex*> vertices;
        bool is_leaf_;
    private:
        GDALDataset* ds_;
        GDALDataset* ds_detailed_;
        OGRLayer* comparison_layer_;
        OGRLayer* tsezne_layer_;
        OGRLayer* unknown_layer_;
        MapService* map_service_;

        std::uniform_real_distribution<double>* random_distribution_x;
        std::uniform_real_distribution<double>* random_distribution_y;
        std::default_random_engine random_engine_;

        int depth_;
        int id_;
        int parent_id_;
        childRegion own_region_;
};