#pragma once

#include "ros/ros.h"
#include <ros/package.h>
#include "gdal/ogrsf_frmts.h"
#include <GeographicLib/Geodesic.hpp>
#include "usv_map/intersect.h"
#include "usv_map/distance.h"
#include "unordered_map"
#include "boost/filesystem.hpp"

enum LayerID {COLLISION, CAUTION, VORONOI};

class MapService {
    public:
        MapService(std::string mission_region);
        MapService(GDALDataset* ds);
        bool intersects(OGRGeometry* geom, LayerID layer_id);
        double distance(double lon,double lat,LayerID layer_id,double max_distance=-1);
        double voronoi_field(double lon, double lat);
        std::pair<OGRPoint, OGRPoint> getMapExtent();
        GDALDataset* getDataset();
    private:
        GDALDataset* ds_;
        GDALDataset* ds_in_mem_;
        GDALDriver* driver_mem_;
        OGRPoint distance_point_;

        //Mission region extent
        OGRPoint lower_left_;
        OGRPoint upper_right_;


        //Parameters
        double alpha_;
        double default_saturation_;
};