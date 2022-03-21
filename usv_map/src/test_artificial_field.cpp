#include "ros/ros.h"
#include "usv_map/map_service.h"

std::pair<OGRPoint, OGRPoint> setCutsomExtent(double lon_lower, double lat_lower, double lon_upper, double lat_upper){
    OGRPoint point_lower;
    OGRPoint point_upper;
    point_lower.setX(lon_lower);
    point_lower.setY(lat_lower);

    point_upper.setX(lon_upper);
    point_upper.setY(lat_upper);
    return std::make_pair(point_lower,point_upper);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"test_artificial_field");
    std::string mission_region = "outside_new_york_2";
    std::string debug_path = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region+"/debug/";
    if(!boost::filesystem::exists(debug_path)){
            boost::filesystem::create_directories(debug_path);
    }

    MapService map_service(mission_region);
    double resolution = 0.001;

    //Get mission region extent
    std::pair<OGRPoint, OGRPoint> extent = map_service.getMapExtent();//setCutsomExtent(-73.878529,40.581363,-73.846124,40.623166);//map_service.getMapExtent();
    std::vector<std::pair<double,double>> points_to_check;
    for(double x=extent.first.getX(); x<extent.second.getX(); x+=resolution){
        for(double y=extent.first.getY(); y<extent.second.getY(); y+=resolution){
            points_to_check.push_back(std::make_pair(x,y));
        }
    }

    std::cout << "Points to check: " << points_to_check.size() << std::endl;

    std::ofstream distance_tiles_file(debug_path+"fields.csv");
    distance_tiles_file << "x_center,y_center,distance_obstacle,distance_voronoi,distance_voronoi_field\n";

    int i = 0;
    for(auto it=points_to_check.begin();it!=points_to_check.end();it++){
        if(i%1000==0){
            std::cout << "Points processed: " << i << std::endl;
        }
        OGRPoint point;
        point.setX((*it).first);
        point.setY((*it).second);
        double distance_voronoi= map_service.distance((*it).first,(*it).second,LayerID::VORONOI);
        double distance_obstacle= map_service.distance((*it).first,(*it).second,LayerID::COLLISION);
        double voronoi_field = map_service.voronoi_field((*it).first,(*it).second);
        
        distance_tiles_file<<(*it).first<<","<<(*it).second<<","<<distance_obstacle<<","<<distance_voronoi<<","<<voronoi_field<<"\n";
        i++;
    }
    


}