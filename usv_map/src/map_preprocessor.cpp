#include "usv_map/map_preprocessor.h"


MapPreprocessor::MapPreprocessor(){
    GDALAllRegister();
    driver_sqlite_ = GetGDALDriverManager()->GetDriverByName("SQLite");
    if(driver_sqlite_==NULL){
        throw std::runtime_error("Unable to find SQLite driver");
        exit(1);
    }

    vessel_width_ = 3;
    vessel_length_ = 10;
    vessel_height_ = 3;
    vessel_draft_ = 0.5;


}

void MapPreprocessor::run(std::string mission_region_name, extractorRegion& region){
    std::string mission_path =  ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_name;
    if(!boost::filesystem::exists(mission_path)){
        boost::filesystem::create_directories(mission_path);
    }
    std::string db_path = mission_path+"/region.sqlite";
    GDALDataset* db = driver_sqlite_->Create(db_path.c_str(),0,0,0,GDT_Unknown,NULL);
    extractorVessel vessel(vessel_width_,vessel_length_,vessel_height_,vessel_draft_);

    //Process ENCs of mission region
    ENCExtractor extractor(region,vessel,db);
    extractor.run();

    //Build quadtree
    OGRPoint lower_left_(region.min_lon_,region.min_lat_);
    OGRPoint upper_right_(region.max_lon_,region.max_lat_);
    Quadtree tree(lower_left_,upper_right_,db,mission_region_name,true);

    //Build voronoi skeleton
    MapService map_service(db);
    GeographicLib::Geodesic geod(GeographicLib::Geodesic::WGS84());
    VoronoiSkeletonGenerator vs_gen("collision_dissolved",lower_left_,upper_right_,db,&tree,&map_service,&geod);
    vs_gen.run();
}

void MapPreprocessor::debug(std::string mission_region_name, extractorRegion& region){

    //Check debug parameters
    bool extract_enc, build_quadtree, build_voronoi;
    bool parameter_load_error = false;
    if(!ros::param::get("preprocessor_debug/extract_enc",extract_enc)) parameter_load_error = true;
    if(!ros::param::get("preprocessor_debug/build_quadtree",build_quadtree)) parameter_load_error = true;
    if(!ros::param::get("preprocessor_debug/build_voronoi",build_voronoi)) parameter_load_error = true;

    GDALDataset* ds;
    std::string mission_path =  ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_name;
    if(extract_enc){
        std::cout << "PreProcessor: Extract ENC" << std::endl;
        ds = extractENC(mission_region_name,region);
    } else{
        std::string db_path = mission_path+"/region.sqlite";
        ds = (GDALDataset*) GDALOpenEx(db_path.c_str(),GDAL_OF_VECTOR,NULL,NULL,NULL);
    }
    if(build_quadtree){
        std::cout << "PreProcessor: Build Quadtree" << std::endl;
        buildQuadtree(mission_region_name,region,ds);
    }
    if(build_voronoi){
        std::cout << "PreProcessor: Generate Voronoi" << std::endl;
        generateVoronoi(region,ds);
    }
}

GDALDataset* MapPreprocessor::extractENC(std::string mission_region_name,extractorRegion& region){
    std::string mission_path =  ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_name;
    if(!boost::filesystem::exists(mission_path)){
            boost::filesystem::create_directories(mission_path);
    }
    std::string db_path = mission_path+"/region.sqlite";
    GDALDataset* ds = driver_sqlite_->Create(db_path.c_str(),0,0,0,GDT_Unknown,NULL);
    extractorVessel vessel(vessel_width_,vessel_length_,vessel_height_,vessel_draft_);
    ENCExtractor extractor(region,vessel,ds);
    extractor.run();
    return ds;
}

void MapPreprocessor::buildQuadtree(std::string mission_region_name, extractorRegion& region, GDALDataset* ds){
    OGRPoint lower_left_(region.min_lon_,region.min_lat_);
    OGRPoint upper_right_(region.max_lon_,region.max_lat_);
    Quadtree tree(lower_left_,upper_right_,ds,mission_region_name,true);
}

void MapPreprocessor::generateVoronoi(extractorRegion& region, GDALDataset* ds){
    MapService map_service(ds);
    GeographicLib::Geodesic geod(GeographicLib::Geodesic::WGS84());
    OGRPoint lower_left(region.min_lon_,region.min_lat_);
    OGRPoint upper_right(region.max_lon_,region.max_lat_);
    VoronoiSkeletonGenerator vs_gen("collision_dissolved",lower_left,upper_right,ds,nullptr,&map_service,&geod);
    vs_gen.run();
}