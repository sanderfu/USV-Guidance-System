#include "usv_map/geotf_ros.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"geotf_ros");
    ros::NodeHandle nh;

    geotf::GeodeticConverterServer converter(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration rate(1);
    while (ros::ok())
    {
        rate.sleep();
    }
    ros::shutdown();
}