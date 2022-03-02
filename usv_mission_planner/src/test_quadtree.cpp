#include "usv_mission_planner/quadtree.h"
#include "usv_mission_planner/astar.h"
#include "usv_mission_planner/hybrid_astar.h"
#include "ros/package.h"

#include "gdal/ogrsf_frmts.h"

void pathToGPX(std::string file_name,std::vector<extendedVertex*>& hybrid_path){
    std::string gpx_path = ros::package::getPath("usv_mission_planner")+"/data/gpx/"+file_name+".gpx";
    if(boost::filesystem::exists(gpx_path)){
        std::cout << "GPX file already exists, deleting" << std::endl;
        boost::filesystem::remove(gpx_path);
    }

    GDALDriver* gpx_driver = GetGDALDriverManager()->GetDriverByName("GPX");
    GDALDataset* gpx_ds = gpx_driver->Create(gpx_path.c_str(),0,0,0,GDT_Unknown,NULL);
    OGRLayer* gpx_routes_layer = gpx_ds->CreateLayer("routes",OGRSpatialReference::GetWGS84SRS(),wkbLineString);
    OGRFeature* gpx_route_feature = OGRFeature::CreateFeature(gpx_routes_layer->GetLayerDefn());

    OGRLineString route;
    OGRPoint route_point;
    for (auto waypoint: hybrid_path){
        route_point.setX(waypoint->pose->x());
        route_point.setY(waypoint->pose->y());
        route.addPoint(&route_point);
    }
    gpx_route_feature->SetGeometry(&route);
    gpx_routes_layer->CreateFeature(gpx_route_feature);

    gpx_ds->~GDALDataset();
    std::cout << "Done writing waypoints to gpx " << std::endl;


}
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
    
    quadtree.setStart(-73.901701601459521,40.566665718424396);
    quadtree.setGoal(-73.8443265,40.6415880);
    quadtree.visualize();

    ros::NodeHandle astar_nh("~AStarROS");
    AStarROS astar(astar_nh,quadtree.getGraphManager());
    astar.setStart(-73.901701601459521,40.566665718424396);
    astar.setGoal(-73.8443265,40.6415880);
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

    pathToGPX("test_gpx",hybrid_path);
    
    /*
    ros::Time start_save = ros::Time::now();
    quadtree.save("test_quadtree_2");
    ros::Time done_save = ros::Time::now();
    std::cout << "Time to save: " << ros::Duration(done_save-start_save).toSec() << std::endl;
    */




    
    

    ros::spin();

}