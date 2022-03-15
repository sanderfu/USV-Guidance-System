#pragma once
#include "gdal/ogrsf_frmts.h"
#include "voronoi/jc_voronoi.h"
#include "usv_map/quadtree.h"
#include "usv_map/map_service.h"

class VoronoiSkeletonGenerator{
    public:
        VoronoiSkeletonGenerator(std::string layername, GDALDataset* ds, Quadtree* tree, MapService* map_service, GeographicLib::Geodesic* geod);
        void run();
    private:
        GDALDataset* ds_;
        Quadtree* tree_;
        MapService* map_service_;
        GeographicLib::Geodesic* geod_;
        OGRLayer* in_layer_;
        OGRLayer* voronoi_layer_;


};