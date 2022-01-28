#include "usv_map/map_service.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"map_service_node");
    ros::NodeHandle nh("map_service_node");

    MapServiceServer server(nh);
    ros::spin();

}