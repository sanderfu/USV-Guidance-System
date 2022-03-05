#include "ros/ros.h"
#include "ros/package.h"
#include "gdal/ogrsf_frmts.h"
#include <fstream>
#include "iostream"
#include "usv_map/map_service.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_distance_concept");
    ros::NodeHandle nh("~testQT");
    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.873976 40.577607)";
    
    point_upper.importFromWkt(&wkt_upper);
    
    std::string db_path_ = ros::package::getPath("usv_simulator");
    db_path_.append("/maps/check_db_lite.sqlite");
    std::cout << db_path_ << std::endl;
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    OGRLayer* comparison_layer = ds->GetLayerByName("collision_dissolved");
    OGRFeature* feat;
    double distance = 0.0;
    double min_distance = INFINITY;

    ros::Time start = ros::Time::now();

    //Alternative 3 (Multipolygon)
    min_distance = INFINITY;
    GDALDriver* driver_memory2 = GetGDALDriverManager()->GetDriverByName("Memory");
    GDALDataset* memory_ds2 = driver_memory2->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    OGRLayer* multi_layer = memory_ds2->CreateLayer("multi",comparison_layer->GetSpatialRef(),wkbMultiPolygon);
    OGRFeature* multi_feature = OGRFeature::CreateFeature(multi_layer->GetLayerDefn());
    OGRMultiPolygon multi_poly;
    comparison_layer->ResetReading();
    while((feat = comparison_layer->GetNextFeature()) != NULL){
        multi_poly.addGeometry(feat->GetGeometryRef());
        OGRFeature::DestroyFeature(feat);
    }
    multi_feature->SetGeometry(&multi_poly);
    multi_layer->CreateFeature(multi_feature);

    std::string distance_tiles_path = ros::package::getPath("usv_map")+"/data/debug_distance_concept/distance_tiles.csv";
    
    std::ofstream distance_tiles_file(distance_tiles_path);
    distance_tiles_file << "x_center,y_center,distance_obstacle,distance_voronoi,distance_voronoi_field\n";


    ros::Time start_alt3 = ros::Time::now();
    double lon_lower_left = -74.02806;
    double lat_lower_left = 40.49421;

    double lon_upper_right = -73.72435;
    double lat_upper_right = 40.65096;

    MapService map_client("outside_new_york_mission");


    std::vector<std::pair<double,double>> points_to_check;
    for(double x=lon_lower_left; x<lon_upper_right; x+=0.001){
        for(double y=lat_lower_left; y<lat_upper_right; y+=0.001){
            points_to_check.push_back(std::make_pair(x,y));
        }
    }
    std::cout << "Points to check: " << points_to_check.size() << std::endl;
    //#pragma omp parallel for
    int i = 0;
    for(auto it=points_to_check.begin();it!=points_to_check.end();it++){
        if(i%1000==0){
            std::cout << "Points processed: " << i << std::endl;
        }
        OGRPoint point;
        point.setX((*it).first);
        point.setY((*it).second);
        //double distance = std::min(multi_layer->GetFeature(0)->GetGeometryRef()->Distance(&point),0.005);
        //ros::Time start_distance = ros::Time::now();
        double distance_voronoi= map_client.distance((*it).first,(*it).second,LayerID::VORONOI);
        double distance_obstacle= map_client.distance((*it).first,(*it).second,LayerID::COLLISION);

        double voronoi_field = (0.001/(0.001+distance_obstacle))*(distance_voronoi/(distance_voronoi+distance_obstacle))*(pow(distance_obstacle-0.01,2)/pow(0.01,2));
        //ros::Time stop_distance = ros::Time::now();
        //std::cout << "Time to check distance: " << ros::Duration(stop_distance-start_distance).toSec() << std::endl;
        //#pragma omp critical
            distance_tiles_file<<(*it).first<<","<<(*it).second<<","<<distance_obstacle<<","<<distance_voronoi<<","<<voronoi_field<<"\n";
        i++;
    }

    ros::Time end_alt3 = ros::Time::now();
    distance_tiles_file.close();
    std::cout << "Minimum distance to layer geometry: " << min_distance << std::endl;
    std::cout << "Alt3: Finding minimum distance took: " << ros::Duration(end_alt3-start_alt3).toSec() << std::endl;

    


}