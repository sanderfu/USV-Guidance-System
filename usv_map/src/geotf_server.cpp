#include "usv_map/geotf_server.h"
namespace geotf {

GeodeticConverterServer::GeodeticConverterServer(const ros::NodeHandle& nh): 
nh_(nh),
GeodeticConverterSynchronized(nh) {
    addSyncedFrameByEPSG("WGS84",4326);

    std::vector<double> global_position_vec;
    if(!nh_.getParam("sim_origin",global_position_vec)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
        ros::shutdown();
    }

    addSyncedFrameByENUOrigin("global_enu",global_position_vec[1],global_position_vec[0],0);
    frame_conversion_srv_ = nh_.advertiseService("/map/frame_conversion",&GeodeticConverterServer::frameConversion,this);
    add_frame_srv_ = nh_.advertiseService("/map/add_frame",&GeodeticConverterServer::addFrame,this);
}

bool GeodeticConverterServer::frameConversion(usv_map::frame_conversion::Request& req, usv_map::frame_conversion::Response& res){
    if(!canConvertSynced(req.from_frame,req.to_frame)){
        ROS_WARN_STREAM("Illegal frame conversion attempted");
        return false;
    }
    Eigen::Vector3d to_vec;
    convertSynced(req.from_frame,pointToVector(req.from_point),req.to_frame,&to_vec);
    res.to_point = vectorToPoint(to_vec);
    return true;
}

bool GeodeticConverterServer::addFrame(usv_map::add_frame::Request& req, usv_map::add_frame::Response& res){
    if(hasFrameSynced(req.frame_name) && !req.replace){
        ROS_WARN_STREAM("Frame name already in use");
        return false;
    } else if (req.replace){
        ROS_WARN_STREAM("Replacing frame");
        removeSyncedFrame(req.frame_name);
    }
    addSyncedFrameByENUOrigin(req.frame_name,req.longitude,req.latitude,req.altitude);
    return true;
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


}//End namespace geotf
