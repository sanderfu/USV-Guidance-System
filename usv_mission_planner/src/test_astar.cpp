#include "usv_mission_planner/astar.h"
#include "usv_map/map_service.h"
#include "usv_map/quadtree.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_astar");
    ros::NodeHandle nh;

    GDALAllRegister();
    std::string map_name = "atloy_fixed_divisor_qt";
    MapService map_service(map_name);
    std::pair<OGRPoint,OGRPoint> map_extent = map_service.getMapExtent();
    Quadtree tree(map_extent.first,map_extent.second,map_service.getDataset(),map_service.getDetailedDataset(),map_name,&map_service,false);

    AStar astar_alg(tree.getGraphManager(),&map_service,map_name);

    tree.setStart(4.82904,61.30309);
    astar_alg.setStart(4.82904,61.30309);
    tree.setGoal(5.02193,61.39766);
    astar_alg.setGoal(5.02193,61.39766);
    std::cout << "Start search" << std::endl;
    std::cout << astar_alg.search() << std::endl;
    astar_alg.dumpSearchBenchmark();
    std::cout << "Search successful" << std::endl;
    tree.saveForVisualization(map_name+"_w_endpoints");

    //Check leaf regions
    Region* start_r = tree.getLeafRegionContaining(4.82904,61.30309);
    char* start_wkt;
    start_r->region_polygon_->exportToWkt(&start_wkt);
    std::cout << start_wkt << std::endl;

    Region* goal_r = tree.getLeafRegionContaining(5.02193,61.39766);
    char* goal_wkt;
    goal_r->region_polygon_->exportToWkt(&goal_wkt);
    std::cout << goal_wkt << std::endl;


    

}