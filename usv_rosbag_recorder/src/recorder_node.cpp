#include "usv_rosbag_recorder/recorder.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"recorder");

    usv_rosbag::RecorderOptions options;
    options.record_all=true;
    options.verbose = false;
    usv_rosbag::Recorder recorder(options);

    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    ros::Duration r(1.0);
    while(ros::ok()){
        r.sleep();
    }
}