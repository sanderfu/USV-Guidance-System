#include "usv_map/geotf_sync.h"

namespace geotf{

GeodeticConverterSynchronized::GeodeticConverterSynchronized(std::string name,bool publish, bool subscribe,const ros::NodeHandle& nh) : 
nh_(nh),
name_(name){
    if(subscribe){
        epsg_frame_sub_ = nh_.subscribe("geotf/epsg_frame",10,&GeodeticConverterSynchronized::EPSGFrameCb,this);
        enu_frame_sub_ = nh_.subscribe("geotf/enu_frame",10,&GeodeticConverterSynchronized::ENUFrameCb,this);
        remove_frame_sub_ = nh_.subscribe("geotf/remove_frame",10,&GeodeticConverterSynchronized::removeFrameCb,this);
    }

    if(publish){
        epsg_frame_pub_ = nh_.advertise<usv_map::EPSG>("geotf/epsg_frame",10,true);
        enu_frame_pub_ = nh_.advertise<usv_map::ENU>("geotf/enu_frame",10,true);
        remove_frame_pub_ = nh_.advertise<usv_map::RemoveFrame>("geotf/remove_frame",10,false);
    }
}

void GeodeticConverterSynchronized::addSyncedFrameByEPSG(std::string name, int code){
    usv_map::EPSG msg;
    msg.frame_name = name;
    msg.epsg_code = code;
    epsg_frame_pub_.publish(msg);
}

void GeodeticConverterSynchronized::addSyncedFrameByENUOrigin(std::string name, double lat, double lon, double alt, bool replace){
    usv_map::ENU msg;
    msg.frame_name = name;
    msg.lon = lon;
    msg.lat = lat;
    msg.alt = alt;
    msg.replace = replace;
    enu_frame_pub_.publish(msg);
}

bool GeodeticConverterSynchronized::convertSynced(std::string input_frame, const Eigen::Vector3d &input, const std::string &output_frame, Eigen::Vector3d *output){
    return convert(input_frame,input,output_frame,output);
}

bool GeodeticConverterSynchronized::canConvertSynced(std::string input_frame,std::string output_frame){
    return canConvert(input_frame,output_frame);
}

bool GeodeticConverterSynchronized::hasFrameSynced(std::string frame){
    return hasFrame(frame);
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
    if(hasFrame(msg.frame_name) && !msg.replace){
        ROS_WARN_STREAM("Attempted to add existing frame: " << msg.frame_name);
    } else if(msg.replace){
        removeFrame(msg.frame_name);
    }
    addFrameByENUOrigin(msg.frame_name,msg.lat,msg.lon,msg.alt);
}

void GeodeticConverterSynchronized::removeFrameCb(const usv_map::RemoveFrame& msg){
    removeFrame(msg.frame_name);
}
}