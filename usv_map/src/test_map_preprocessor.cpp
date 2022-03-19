#include "usv_map/map_preprocessor.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"map_preprocessor_node");
    ros::NodeHandle nh("map_preprocessor_node");

    MapPreprocessor preprocessor;
    extractorRegion r(-74.09595,40.50455,-73.72809,40.64828);
    //xtractorRegion r(9.00821,63.43351,10.4405,63.7147);
    ros::Time start_preprocess = ros::Time::now();
    preprocessor.debug("outside_new_york_2",r);
    ros::Time end_preprocess = ros::Time::now();
    std::cout << "Time to preprocess: " << ros::Duration(end_preprocess-start_preprocess).toSec() << std::endl;

}