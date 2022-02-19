#include "usv_mission_planner/quadtree.h"
#include "usv_mission_planner/astar.h"
#include "usv_mission_planner/hybrid_astar.h"
#include "ros/package.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"test_quad");
    
    OGRPoint point_lower;
    const char* wkt_lower = "POINT(-74.02352 40.49976)";

    OGRPoint point_upper;
    const char* wkt_upper = "POINT(-73.92878 40.57358)";
    
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
    quadtree.load("test_quadtree");
    ros::Time done_load = ros::Time::now();
    std::cout << "Time to load: " << ros::Duration(done_load-start_load).toSec() << std::endl;
    

    quadtree.setStart(-74.00305,40.54273);
    quadtree.setGoal(-73.94537,40.55109);

    ros::NodeHandle astar_nh("~AStarROS");
    AStarROS astar(astar_nh,quadtree.getGraphManager());
    astar.setStart(-74.00305,40.54273);
    astar.setGoal(-73.94537,40.55109);
    astar.search();

    std::vector<Vertex*> path = astar.getPath();
    std::cout << "Path length: " << path.size() << std::endl;
    astar.visualize();

    ros::NodeHandle hybrid_astar_nh("~HybridAStarROS");
    ModelLibrary::Viknes830 viknes;
    MapServiceClient map_client(&hybrid_astar_nh);
    ROS_INFO_STREAM("Sleep for 5s waiting for map server");
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Sleep done");
    HybridAStarROS hybrid_astar(hybrid_astar_nh,&quadtree,&viknes,&map_client);
    hybrid_astar.setStart(-73.972908,40.523693,0);
    hybrid_astar.setGoal(-73.974206,40.542943,0);
    hybrid_astar.search();

    std::vector<extendedVertex*> hybrid_path = hybrid_astar.getPath();
    std::cout << "Path length: " << hybrid_path.size() << std::endl;
    hybrid_astar.visualize();

    
    ros::Time start_save = ros::Time::now();
    quadtree.save("test_quadtree_saveissue");
    ros::Time done_save = ros::Time::now();
    std::cout << "Time to save: " << ros::Duration(done_save-start_save).toSec() << std::endl;
    

    quadtree.visualize();
    quadtree.testGetRegion(-73.944971,40.546511);

    ros::spin();

}