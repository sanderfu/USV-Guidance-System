#include "usv_map/add_frame.h"
#include "usv_map/frame_conversion.h"
#include "Eigen/Sparse"

#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_geotf_ros");
    ros::NodeHandle nh("test_geotf_ros");

    ros::ServiceClient client_add_frame = nh.serviceClient<usv_map::add_frame>("/map/add_frame");
    ros::ServiceClient client_conversion = nh.serviceClient<usv_map::frame_conversion>("/map/frame_conversion");

    //Test adding a frame
    usv_map::add_frame srv;
    srv.request.frame_name="enu1";
    srv.request.latitude = 40.5612;
    srv.request.longitude = -73.9761;
    srv.request.altitude = 0;
    if(!client_add_frame.call(srv)){
        ROS_ERROR_STREAM("Failed to add frame!");
    }

    //Test frame conversion
    //Warning: the format of the position MUST be LonLatAlt
    usv_map::frame_conversion srv_conv;
    srv_conv.request.from_point.x = -73.9761; //Longitude (W/E)
    srv_conv.request.from_point.y = 40.5612;
    srv_conv.request.from_point.z = 0;
    srv_conv.request.from_frame = "WGS84";
    srv_conv.request.to_frame = "enu1";
    if(!client_conversion.call(srv_conv)){
        ROS_ERROR_STREAM("Failed to convert coordinates!");
    } else{
        std::cout << "Converted to: " << srv_conv.response.to_pose.pose.position.x << " " << srv_conv.response.to_pose.pose.position.y << std::endl;
    }



}