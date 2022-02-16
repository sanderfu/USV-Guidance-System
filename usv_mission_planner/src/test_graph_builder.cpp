#include "usv_mission_planner/graph_builder.h"
#include "ros/package.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_quad");
    
    OGRPoint point_lower;
    const char* wkt_lower = "POINT(-73.95258 40.54013)";

    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.93830 40.56061)";
    
    point_lower.importFromWkt(&wkt_lower);
    point_upper.importFromWkt(&wkt_upper);
    
    std::string db_path_ = ros::package::getPath("usv_simulator");
    db_path_.append("/maps/check_db.sqlite");
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);

    ros::NodeHandle nh("~testQT");
    QuadtreeROS quadtree(nh,point_lower,point_upper,ds);
    std::cout << "Sizeof quadtree: " << sizeof(quadtree) << std::endl;
    //quadtree.load("quadtree");
    quadtree.visualize();
    quadtree.testGetRegion(-73.944971,40.546511);
    quadtree.save("quadtree2");

    //ros::spin();

}