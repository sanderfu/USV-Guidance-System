#pragma once
#include "enc_extraction/enc_extract_lib.h"
#include "usv_map/skeleton_generator.h"
#include "usv_map/quadtree.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "boost/filesystem.hpp"

class MapPreprocessor{
    public:
        MapPreprocessor();
        void run(std::string mission_region_name, extractorRegion& region);
        void debug(std::string mission_region_name, extractorRegion& region);
    private:
        GDALDriver* driver_sqlite_;

        //Vessel data (TODO: Load from parameter server)
        double vessel_width_;
        double vessel_length_;
        double vessel_height_;
        double vessel_draft_;

        GDALDataset* extractENC(std::string mission_region_name, extractorRegion& region);
        void buildQuadtree(std::string mission_region_name, extractorRegion& region, GDALDataset* ds, bool build);
        void generateVoronoi(extractorRegion& region, GDALDataset* ds);
};