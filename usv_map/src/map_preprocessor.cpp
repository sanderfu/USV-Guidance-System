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

    ENCExtractor extractor(region,vessel,db);
    extractor.run();
}