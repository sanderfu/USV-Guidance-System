#include "usv_mission_planner/quadtree.h"
#include "usv_mission_planner/astar.h"
#include "usv_mission_planner/hybrid_astar.h"
#include "ros/package.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_quad");
    
    OGRPoint point_lower;
    //const char* wkt_lower = "POINT(-73.98275 40.50820)";
    const char* wkt_lower = "POINT(-74.02454 40.49856)";

    OGRPoint point_upper;
    //const char* wkt_upper = "POINT(-73.90424 40.58268)";
    const char* wkt_upper = "POINT(-73.72638 40.64910)";
    
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
    
    
    
    quadtree.setStart(-73.999927,40.590175);
    quadtree.setGoal(-73.839776,40.641721);
    quadtree.visualize();

    ros::NodeHandle astar_nh("~AStarROS");
    AStarROS astar(astar_nh,quadtree.getGraphManager());
    astar.setStart(-73.999927,40.590175);
    astar.setGoal(-73.839776,40.641721);
    ros::Time start_astar = ros::Time::now();
    astar.search();
    ros::Time end_astar = ros::Time::now();
    ROS_INFO_STREAM("AStar search took " << ros::Duration(end_astar-start_astar).toSec());
    

    std::vector<Vertex*> path = astar.getPath();
    std::cout << "Path length: " << path.size() << std::endl;
    astar.visualize();
    
    
    
    ROS_INFO_STREAM("Start Hybrid A*");
    ros::NodeHandle hybrid_astar_nh("~HybridAStarROS");
    ModelLibrary::Viknes830 viknes;
    ROS_INFO_STREAM("Sleep for 5s waiting for map server");
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Sleep done");
    HybridAStarROS hybrid_astar(hybrid_astar_nh,&quadtree,&viknes);
    hybrid_astar.setStart(-73.999927,40.590175,-M_PI);
    hybrid_astar.setGoal(-73.8443265,40.6415880,0);
    hybrid_astar.search();
    
    

    std::vector<extendedVertex*> hybrid_path = hybrid_astar.getPath();
    std::cout << "Path length: " << hybrid_path.size() << std::endl;
    hybrid_astar.visualize();
    
    
    
    /*
    ros::Time start_save = ros::Time::now();
    quadtree.save("test_quadtree_2");
    ros::Time done_save = ros::Time::now();
    std::cout << "Time to save: " << ros::Duration(done_save-start_save).toSec() << std::endl;
    */
    
    

    
    quadtree.visualize();
    double lon, lat;
    quadtree.testGetRegion(-73.944286,40.536814);
    
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