#include "usv_map/geotf_sync.h"

namespace geotf{

GeodeticConverterSynchronized::GeodeticConverterSynchronized(ros::NodeHandle& nh) : nh_(nh){
    epsg_frame_sub_ = nh_.subscribe("geotf/epsg_frame",1,&GeodeticConverterSynchronized::EPSGFrameCb,this);
    enu_frame_sub_ = nh_.subscribe("geotf/enu_frame",1,&GeodeticConverterSynchronized::ENUFrameCb,this);
    remove_frame_sub_ = nh_.subscribe("geotf/remove_frame",1,&GeodeticConverterSynchronized::removeFrameCb,this);

    epsg_frame_pub_ = nh_.advertise<usv_map::EPSG>("geotf/epsg_frame",1,false);
    enu_frame_pub_ = nh_.advertise<usv_map::ENU>("geotf/enu_frame",1,false);
    remove_frame_pub_ = nh_.advertise<usv_map::RemoveFrame>("geotf/remove_frame",1,false);
}

void GeodeticConverterSynchronized::addSyncedFrameByEPSG(std::string name, int code){
    usv_map::EPSG msg;
    msg.frame_name = name;
    msg.epsg_code = code;
    epsg_frame_pub_.publish(msg);
}

void GeodeticConverterSynchronized::addSyncedFrameByENUOrigin(std::string name, double lon, double lat, double alt){
    usv_map::ENU msg;
    msg.frame_name = name;
    msg.lon = lon;
    msg.lat = lat;
    msg.alt = alt;
    enu_frame_pub_.publish(msg);
}

bool GeodeticConverterSynchronized::convertSynced(std::string input_frame, const Eigen::Vector3d &input, const std::string &output_frame, Eigen::Vector3d *output){
    return convert(input_frame,input,output_frame,output);
}

void GeodeticConverterSynchronized::removeSyncedFrame(std::string name){
    usv_map::RemoveFrame msg;
    msg.frame_name = name;
    remove_frame_pub_.publish(msg);
}

void GeodeticConverterSynchronized::EPSGFrameCb(const usv_map::EPSG& msg){
    addFrameByEPSG(msg.frame_name,msg.epsg_code);
}

void GeodeticConverterSynchronized::ENUFrameCb(const usv_map::ENU& msg){
    addFrameByENUOrigin(msg.frame_name,msg.lat,msg.lon,msg.alt);
}

void GeodeticConverterSynchronized::removeFrameCb(const usv_map::RemoveFrame& msg){
    removeFrame(msg.frame_name);
}
}