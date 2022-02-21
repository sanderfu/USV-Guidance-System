#include "usv_mission_planner/quadtree.h"
#include "usv_mission_planner/astar.h"
#include "usv_mission_planner/hybrid_astar.h"
#include "ros/package.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_quad");
    
    OGRPoint point_lower;
    const char* wkt_lower = "POINT(-73.98275 40.50820)";

    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.90424 40.58268)";
    
    point_lower.importFromWkt(&wkt_lower);
    point_upper.importFromWkt(&wkt_upper);
    
    std::string db_path_ = ros::package::getPath("usv_simulator");
    db_path_.append("/maps/check_db.sqlite");
    GDALAllRegister();
    GDALDataset* ds = (GDALDataset*) GDALOpenEx(db_path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);

    ros::NodeHandle nh("~testQT");
    QuadtreeROS quadtree(nh,point_lower,point_upper,ds,false);
    
    ros::Time start_load = ros::Time::now();
    quadtree.load("test_quadtree");
    ros::Time done_load = ros::Time::now();
    std::cout << "Time to load: " << ros::Duration(done_load-start_load).toSec() << std::endl;

    //quadtree.setStart(-74.00305,40.54273);
    //quadtree.setGoal(-73.94537,40.55109);

    /*
    ros::NodeHandle astar_nh("~AStarROS");
    AStarROS astar(astar_nh,quadtree.getGraphManager());
    astar.setStart(-74.00305,40.54273);
    astar.setGoal(-73.94537,40.55109);
    astar.search();

    std::vector<Vertex*> path = astar.getPath();
    std::cout << "Path length: " << path.size() << std::endl;
    astar.visualize();
    */
    ros::NodeHandle hybrid_astar_nh("~HybridAStarROS");
    ModelLibrary::Viknes830 viknes;
    MapServiceClient map_client(&hybrid_astar_nh);
    ROS_INFO_STREAM("Sleep for 5s waiting for map server");
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Sleep done");
    HybridAStarROS hybrid_astar(hybrid_astar_nh,&quadtree,&viknes,&map_client);
    hybrid_astar.setStart(-73.972908,40.523693,0);
    hybrid_astar.setGoal(-73.918452,40.568992,0);
    hybrid_astar.search();

    std::vector<extendedVertex*> hybrid_path = hybrid_astar.getPath();
    std::cout << "Path length: " << hybrid_path.size() << std::endl;
    hybrid_astar.visualize();
    
    /*
    ros::Time start_save = ros::Time::now();
    quadtree.save("test_quadtree");
    ros::Time done_save = ros::Time::now();
    std::cout << "Time to save: " << ros::Duration(done_save-start_save).toSec() << std::endl;
    */

    quadtree.visualize();
    double lon, lat;
    quadtree.testGetRegion(-73.9276,40.5626);
    /*
    while(true && ros::ok()){
        std::cout << "lon: " << std::endl;
        std::cin >> lon;
        std::cout << "lat: " << std::endl;
        std::cin >> lat; 
        std::cout << lon << " " << lat << std::endl;
        quadtree.testGetRegion(lon,lat);
    }
    */

    ros::spin();

}