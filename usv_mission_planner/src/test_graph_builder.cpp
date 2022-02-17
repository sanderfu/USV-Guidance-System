#include "usv_mission_planner/graph_builder.h"
#include "ros/package.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_quad");
    
    OGRPoint point_lower;
    const char* wkt_lower = "POINT(-74.02626 40.49745)";

    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.72619 40.64929)";
    
    point_lower.importFromWkt(&wkt_lower);
    point_upper.importFromWkt(&wkt_upper);
    
    std::string db_path_ = ros::package::getPath("usv_simulator");
    db_path_.append("/maps/check_db.sqlite");
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);

    ros::NodeHandle nh("~testQT");
    QuadtreeROS quadtree(nh,point_lower,point_upper,ds,false);
    //std::cout << "Sizeof quadtree: " << sizeof(quadtree) << std::endl;
    ros::Time start_load = ros::Time::now();
    quadtree.load("quadtree");
    ros::Time done_load = ros::Time::now();
    std::cout << "Time to load: " << ros::Duration(done_load-start_load).toSec() << std::endl;


    quadtree.visualize();
    quadtree.testGetRegion(-73.944971,40.546511);
    ros::Time start_save = ros::Time::now();
    quadtree.save("quadtree3");
    ros::Time done_save = ros::Time::now();
    std::cout << "Time to save: " << ros::Duration(done_save-start_save).toSec() << std::endl;

    //ros::spin();

}