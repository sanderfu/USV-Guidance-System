#include "ros/ros.h"
#include "ros/package.h"
#include "gdal/ogrsf_frmts.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_distance_concept");
    ros::NodeHandle nh("~testQT");
    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.873976 40.577607)";
    
    point_upper.importFromWkt(&wkt_upper);
    
    std::string db_path_ = ros::package::getPath("voroni");
    db_path_.append("/data/test_map/check_db_detailed.sqlite");
    std::cout << db_path_ << std::endl;
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    OGRLayer* comparison_layer = ds->GetLayerByName("collision_dissolved");
    OGRFeature* feat;
    double distance = 0.0;
    double min_distance = INFINITY;

    //0.01 degree is 1.11 km approx
    ros::Time start = ros::Time::now();

    //Alternative 1
    ros::Time start_alt1 = ros::Time::now();
    for(int i =0; i<2; i++){
        comparison_layer->ResetReading();
        while((feat = comparison_layer->GetNextFeature()) != NULL){
            distance = feat->GetGeometryRef()->Distance(&point_upper);
            if (distance<min_distance){
                min_distance=distance;
            }
            OGRFeature::DestroyFeature(feat);
        }
    }
    
    ros::Time end_alt1 = ros::Time::now();
    std::cout << "Minimum distance to layer geometry: " << min_distance << std::endl;
    std::cout << "Alt1: Finding minimum distance took: " << ros::Duration(end_alt1-start_alt1).toSec() << std::endl;
    //Alternative 2
    min_distance=INFINITY;
    //std::string test_path = ros::package::getPath("usv_map")+"/test.sqlite";
    GDALDriver* driver_memory = GetGDALDriverManager()->GetDriverByName("Memory");
    GDALDataset* memory_ds = driver_memory->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    OGRLayer* circle_layer = memory_ds->CreateLayer("circle",comparison_layer->GetSpatialRef());
    OGRFeature* circle_feature = OGRFeature::CreateFeature(circle_layer->GetLayerDefn());
    ros::Time start_alt2 = ros::Time::now();
    for(int i=0; i<2; i++){
        circle_feature->SetGeometry(point_upper.Buffer(0.01,100));
        circle_layer->CreateFeature(circle_feature);

        OGRLayer* intersect_layer = memory_ds->CreateLayer("intersect_layer",comparison_layer->GetSpatialRef());
        comparison_layer->Intersection(circle_layer,intersect_layer);
        //std::cout << "Features in intersection layer" << intersect_layer->GetFeatureCount() <<std::endl;

    
        intersect_layer->ResetReading();
        while((feat = intersect_layer->GetNextFeature()) != NULL){
            distance = feat->GetGeometryRef()->Distance(&point_upper);
            if (distance<min_distance){
                min_distance=distance;
            }
            OGRFeature::DestroyFeature(feat);
        }
    }

    ros::Time end_alt2 = ros::Time::now();
    std::cout << "Minimum distance to layer geometry: " << min_distance << std::endl;
    std::cout << "Alt2: Finding minimum distance took: " << ros::Duration(end_alt2-start_alt2).toSec() << std::endl;



}