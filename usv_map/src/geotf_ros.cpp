#include "usv_map/geotf_ros.h"
namespace geotf {

GeodeticConverterServer::GeodeticConverterServer(const ros::NodeHandle& nh): nh_(nh) {
    addFrameByEPSG("WGS84",4326);
    addFrameByENUOrigin("global_enu",40.5612,-73.9761,0);
    frame_conversion_srv_ = nh_.advertiseService("/map/frame_conversion",&GeodeticConverterServer::frameConversion,this);
    add_frame_srv_ = nh_.advertiseService("/map/add_frame",&GeodeticConverterServer::addFrame,this);
}

bool GeodeticConverterServer::frameConversion(usv_map::frame_conversion::Request& req, usv_map::frame_conversion::Response& res){
    if(!canConvert(req.from_frame,req.to_frame)){
        ROS_WARN_STREAM("Illegal frame conversion attempted");
        return false;
    }
    Eigen::Vector3d to_vec;
    convert(req.from_frame,pointToVector(req.from_point),req.to_frame,&to_vec);
    res.to_point = vectorToPoint(to_vec);
    return true;
}

bool GeodeticConverterServer::addFrame(usv_map::add_frame::Request& req, usv_map::add_frame::Response& res){
    if(hasFrame(req.frame_name)){
        ROS_WARN_STREAM("Frame name already in use");
        return false;
    }
    addFrameByENUOrigin(req.frame_name,req.latitude,req.longitude,req.altitude);
    return true;
}

GeodeticConverterClient::GeodeticConverterClient(ros::NodeHandle* nh): nh_(nh){
    add_frame_client_ = nh_->serviceClient<usv_map::add_frame>("/map/add_frame");
    convert_point_client_ = nh_->serviceClient<usv_map::frame_conversion>("/map/frame_conversion");
}

void GeodeticConverterClient::addFrame(std::string name, double lon, double lat, double alt){
    usv_map::add_frame srv;
    srv.request.frame_name=name;
    srv.request.latitude = lat;
    srv.request.longitude = lon;
    srv.request.altitude = alt;
    if(!add_frame_client_.call(srv)){
        ROS_ERROR_STREAM("Failed to add frame!");
    }
}

void GeodeticConverterClient::convert(std::string& from_frame, Eigen::Vector3d& from_vec, std::string& to_frame, Eigen::Vector3d& to_vec){
    usv_map::frame_conversion srv_conv;
    srv_conv.request.from_point = vectorToPoint(from_vec);
    srv_conv.request.from_frame = from_frame;
    srv_conv.request.to_frame = to_frame;
    if(!convert_point_client_.call(srv_conv)){
        ROS_ERROR_STREAM("Failed to convert coordinates!");
    } else{
        std::cout << "Converted to: " << srv_conv.response.to_point.x << " " << srv_conv.response.to_point.y << std::endl;
    }
}

Eigen::Vector3d pointToVector(geometry_msgs::Point& point){
    Eigen::Vector3d vec;
    vec(0) = point.x;
    vec(1) = point.y;
    vec(2) = point.z;
    return vec;
}

geometry_msgs::Point vectorToPoint(Eigen::Vector3d& vec){
    geometry_msgs::Point point;
    point.x = vec(0);
    point.y = vec(1);
    point.z = vec(2);
    return point;
}


}//End namespace geotf_ros
