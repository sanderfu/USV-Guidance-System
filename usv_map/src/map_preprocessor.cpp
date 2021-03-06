#include "usv_map/map_preprocessor.h"


MapPreprocessor::MapPreprocessor(){
    GDALAllRegister();
    driver_sqlite_ = GetGDALDriverManager()->GetDriverByName("SQLite");
    if(driver_sqlite_==NULL){
        throw std::runtime_error("Unable to find SQLite driver");
        exit(1);
    }

    //TODO: These parameters should be in configuration file
    vessel_width_ = 3;
    vessel_length_ = 10;
    vessel_height_ = 3;
    vessel_draft_ = 0.5;


}

/**
 * @brief The main procedure of the map preprocessor. Builds and stores a mission region.
 * 
 * @param mission_region_name 
 * @param region 
 */
void MapPreprocessor::run(std::string mission_region_name, extractorRegion& region){
    GDALDataset* ds;
    GDALDataset* ds_detailed;
    std::pair<GDALDataset*, GDALDataset*> datasets;
    std::string mission_path =  ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_name;

    //Process ENCs of mission region
    datasets = extractENC(mission_region_name,region);
    ds = datasets.first;
    ds_detailed = datasets.second;

    //Build regioin framed quadtree
    buildQuadtree(mission_region_name,region,ds,ds_detailed,true);

    //Build voronoi skeleton
    generateVoronoi(mission_region_name,region,ds,ds_detailed);
}

/**
 * @brief Procedure for optionally building only part of the mission region. 
 * 
 * @param mission_region_name 
 * @param region 
 */
void MapPreprocessor::debug(std::string mission_region_name, extractorRegion& region){

    //Check debug parameters
    bool extract_enc, build_quadtree, build_voronoi;
    bool parameter_load_error = false;
    if(!ros::param::get("preprocessor_debug/extract_enc",extract_enc)) parameter_load_error = true;
    if(!ros::param::get("preprocessor_debug/build_quadtree",build_quadtree)) parameter_load_error = true;
    if(!ros::param::get("preprocessor_debug/build_voronoi",build_voronoi)) parameter_load_error = true;

    GDALDataset* ds;
    GDALDataset* ds_detailed;
    std::pair<GDALDataset*, GDALDataset*> datasets;
    std::string mission_path =  ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_name;
    if(extract_enc){
        std::cout << "PreProcessor: Extract ENC" << std::endl;
        ros::Time start = ros::Time::now();
        datasets = extractENC(mission_region_name,region);
        ds = datasets.first;
        ds_detailed = datasets.second;
        std::cout << "Preprocessing ENC took " << ros::Duration(ros::Time::now()-start).toSec() << " [s]" << std::endl;
    } else{
        std::string db_path = mission_path+"/region.sqlite";
        ds = (GDALDataset*) GDALOpenEx(db_path.c_str(),GDAL_OF_VECTOR | GDAL_OF_UPDATE,NULL,NULL,NULL);
        
        std::string db_detailed_path = mission_path+"/region_detailed.sqlite";
        ds_detailed = (GDALDataset*) GDALOpenEx(db_detailed_path.c_str(),GDAL_OF_VECTOR,NULL,NULL,NULL);

    }
    
    std::cout << "PreProcessor: Build Quadtree" << std::endl;
    buildQuadtree(mission_region_name,region,ds,ds_detailed,build_quadtree);

    if(build_voronoi){
        std::cout << "PreProcessor: Generate Voronoi" << std::endl;
        generateVoronoi(mission_region_name,region,ds,ds_detailed);
    }
}

/**
 * @brief Extract detailed and overview spatial databases from ENC tiles covering mission region.
 * 
 * @remark Assuming ENCs have been ingested following the procedure in enc_extract_lib, this
 * procedure takes care of selecting which ENCs are relevant. If ENCs are not ingested and stored
 * in the enc_extract_lib then this function will fail.
 * 
 * @param mission_region_name 
 * @param region 
 * @return std::pair<GDALDataset*, GDALDataset*> The overview (first) and the detailed (second) spatial databases.
 */
std::pair<GDALDataset*, GDALDataset*> MapPreprocessor::extractENC(std::string mission_region_name,extractorRegion& region){
    std::string mission_path =  ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_name;
    if(!boost::filesystem::exists(mission_path)){
            boost::filesystem::create_directories(mission_path);
    }
    std::string db_path = mission_path+"/region.sqlite";
    GDALDataset* ds = driver_sqlite_->Create(db_path.c_str(),0,0,0,GDT_Unknown,NULL);

    std::string db_detailed_path = mission_path+"/region_detailed.sqlite";
    GDALDataset* ds_detailed = driver_sqlite_->Create(db_detailed_path.c_str(),0,0,0,GDT_Unknown,NULL);

    extractorVessel vessel(vessel_width_,vessel_length_,vessel_height_,vessel_draft_);
    ENCExtractor extractor(region,vessel,ds,ds_detailed);
    extractor.run();
    return std::make_pair(ds,ds_detailed);
}

/**
 * @brief Build regional framed quadtree of mission region
 * 
 * @param mission_region_name 
 * @param region 
 * @param ds 
 * @param ds_detailed 
 * @param build 
 */
void MapPreprocessor::buildQuadtree(std::string mission_region_name, extractorRegion& region, GDALDataset* ds,GDALDataset* ds_detailed,bool build){
    MapService map_service(ds, ds_detailed);
    OGRPoint lower_left_(region.min_lon_,region.min_lat_);
    OGRPoint upper_right_(region.max_lon_,region.max_lat_);
    Quadtree tree(lower_left_,upper_right_,ds,ds_detailed,mission_region_name,&map_service,build);
}

/**
 * @brief Build Voronoi skeleton of mission region free-space.
 * 
 * @param mission_region_name 
 * @param region 
 * @param ds 
 * @param ds_detailed 
 */
void MapPreprocessor::generateVoronoi(std::string mission_region_name,extractorRegion& region, GDALDataset* ds, GDALDataset* ds_detailed){
    MapService map_service(ds,ds_detailed);
    GeographicLib::Geodesic geod(GeographicLib::Geodesic::WGS84());
    OGRPoint lower_left(region.min_lon_,region.min_lat_);
    OGRPoint upper_right(region.max_lon_,region.max_lat_);
    VoronoiSkeletonGenerator vs_gen("collision_dissolved",lower_left,upper_right,ds,mission_region_name,nullptr,&map_service,&geod);
    vs_gen.run();
}