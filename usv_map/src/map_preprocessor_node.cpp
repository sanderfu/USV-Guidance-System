#include "usv_map/map_preprocessor.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"map_preprocessor_node");
    ros::NodeHandle nh("map_preprocessor_node");

    MapPreprocessor preprocessor;
    extractorRegion r(-74.02483,40.49961,-73.72579,40.64967);
    preprocessor.run("test_mission_region",r);
    ros::spin();

}