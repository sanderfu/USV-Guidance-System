#pragma once
#include "gdal/ogrsf_frmts.h"
#include "usv_map/jc_voronoi.h"
#include "usv_map/quadtree.h"
#include "usv_map/map_service.h"
#include "boost/unordered_map.hpp"
#include "set"

class VoronoiSkeletonGenerator{
    public:
        VoronoiSkeletonGenerator(std::string layername, OGRPoint& lower_left, OGRPoint& upper_right, GDALDataset* ds, Quadtree* tree, MapService* map_service, GeographicLib::Geodesic* geod);
        void run();
    private:
        GDALDataset* ds_;
        Quadtree* tree_;
        MapService* map_service_;
        GeographicLib::Geodesic* geod_;
        OGRLayer* in_layer_;
        OGRLayer* voronoi_layer_;

        OGRPoint lower_left_;
        OGRPoint upper_right_;

        //Config
        int precision = 1e6;

        //Benchmark tools
        int sites_count_;
        int diagram_edges_count_;
        int pruned_edges_count_;
        int skeleton_edges_count_;

        double total_build_time_;
        double build_skeleton_time_;
        double register_candidate_edges_time_;
        double prune_edges_time_;
        double identify_unique_edges_time_;
        double add_edges_to_dataset_time_;
        std::vector<double> collision_times_;
        std::vector<double> point_in_region_times_;

        void buildSkeleton(jcv_diagram& diagram);
        void registerCandidateEdges(jcv_diagram& diagram, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void pruneEdges(std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void identifyUniqueEdges(std::set<const jcv_edge*>& unique_remaining_edges, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void addEdgesToDataset(std::set<const jcv_edge*>& unique_remaining_edges);


        //Helping function(s)
        bool pointInRegion(double lon, double lat);
        bool collision(const jcv_edge* edge);

        //Debug functions
        double smallestDistanceMeasured(boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void dumpDebug();


};