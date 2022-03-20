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
        double sites_count_;
        double diagram_edges_count_;
        double pruned_edges_count_;

        ros::Time start_build_;
        ros::Time end_build_;
        std::vector<double> get_sites_time_;
        std::vector<double> generate_voronoi_diagram_time_;
        std::vector<double> point_in_region_time_;
        std::vector<double> collision_time_;
        std::vector<double> prune_process_time_;
        std::vector<double> register_unique_edges_time_;
        std::vector<double> add_unique_line_to_voronoi;

        void pruneEdges(jcv_diagram& diagram);
        void registerCandidateEdges(jcv_diagram& diagram, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void removeInvalidEdges(std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void identifyUniqueEdges(std::set<const jcv_edge*>& unique_remaining_edges, std::unordered_map<int,const jcv_edge*>& edge_map, boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);
        void addEdgesToDataset(std::set<const jcv_edge*>& unique_remaining_edges);


        //Helping function(s)
        bool pointInRegion(double lon, double lat);

        //Debug functions
        double smallestDistanceMeasured(boost::unordered_map<std::pair<int,int>,std::vector<int64_t>>& point_map);



};