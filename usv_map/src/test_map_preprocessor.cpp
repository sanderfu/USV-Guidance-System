#include "usv_map/map_preprocessor.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"map_preprocessor_node_2");
    ros::NodeHandle nh("map_preprocessor_node_2");

    MapPreprocessor preprocessor;
    extractorRegion r(5.55656,58.93981,6.38519845244248,59.6649273906081); //stavanger_sauda_ryfylke
    //extractorRegion r(5.55656,58.93981,5.87488,59.14257); //stavanger_sauda_ryfylke_small
    //extractorRegion r(7.637389079067764,62.68744380057604,11.229159081279168,63.73981746846386); //trondheim_sunndalsora
    //extractorRegion r(4.82157,61.28804,5.06143,61.40352); //atloy

    //xtractorRegion r(9.00821,63.43351,10.4405,63.7147);
    ros::Time start_preprocess = ros::Time::now();
    preprocessor.debug("stavanger_sauda_ryfylke",r);
    ros::Time end_preprocess = ros::Time::now();
    std::cout << "Time to preprocess: " << ros::Duration(end_preprocess-start_preprocess).toSec() << std::endl;

}