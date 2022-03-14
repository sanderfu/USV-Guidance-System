#pragma once
#include "enc_extraction/enc_extract_lib.h"
#include "ros/package.h"
#include "boost/filesystem.hpp"

class MapPreprocessor{
    public:
        MapPreprocessor();
        void run(std::string mission_region_name, extractorRegion& region);
    private:
        ENCExtractor* enc_extractor_;
        //VoronoiSkeletonGenerator* voronoi_skeleton_generator_;
        GDALDriver* driver_sqlite_;

        //Vessel data (TODO: Load from parameter server)
        double vessel_width_;
        double vessel_length_;
        double vessel_height_;
        double vessel_draft_;
};