#include "usv_colav/colav.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"colav_node");
    ros::NodeHandle nh;

    Colav colav(nh);
    ros::spin();

}