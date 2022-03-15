#include "usv_map/map_service.h"
#include "ros/topic.h"
#include "std_msgs/Bool.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"map_service_node");
    ros::NodeHandle nh;
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available",nh);
    MapServiceServer server(nh);
    ros::spin();

}