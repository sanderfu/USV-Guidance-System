#include "usv_mission_planner/graph_builder.h"
#include "ros/package.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_quad");
    
    OGRPoint point_lower;
    const char* wkt_lower = "POINT(-73.95690 40.53364)";

    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.93398 40.56666)";
    
    point_lower.importFromWkt(&wkt_lower);
    point_upper.importFromWkt(&wkt_upper);
    
    std::string db_path_ = ros::package::getPath("usv_simulator");
    db_path_.append("/maps/check_db.sqlite");
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);

    ros::NodeHandle nh("~testQT");
    QuadtreeROS quadtree(nh,point_lower,point_upper,ds);
    //quadtree.load("quadtree");
    quadtree.visualize();
    quadtree.save("quadtree");

    ros::spin();

}