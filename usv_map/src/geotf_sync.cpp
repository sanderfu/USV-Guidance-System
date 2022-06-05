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

/**
 * @brief Add a synced frame by EPSG code 
 * 
 * @param name 
 * @param code 
 */
void GeodeticConverterSynchronized::addSyncedFrameByEPSG(std::string name, int code){
    usv_map::EPSG msg;
    msg.frame_name = name;
    msg.epsg_code = code;
    epsg_frame_pub_.publish(msg);
}

/**
 * @brief Add a synced frame by ENU origin
 * 
 * @param name Frame name
 * @param lat Origin latitude
 * @param lon Origin longitude
 * @param alt Origin altitude
 * @param replace Set true if any frame with the same name should be replaced. If not, the new frame can be ignored.
 */
void GeodeticConverterSynchronized::addSyncedFrameByENUOrigin(std::string name, double lat, double lon, double alt, bool replace){
    usv_map::ENU msg;
    msg.frame_name = name;
    msg.lon = lon;
    msg.lat = lat;
    msg.alt = alt;
    msg.replace = replace;
    enu_frame_pub_.publish(msg);
}

/**
 * @brief Function to convert a 3D vector from one frame to another.
 * 
 * @remark Both frames must be well defined in advance.
 * 
 * @param input_frame Input frame name
 * @param input Input vector
 * @param output_frame Output frame name
 * @param output Output vector
 * @return true If conversation succeeded
 * @return false If conversion failed
 */
bool GeodeticConverterSynchronized::convertSynced(std::string input_frame, const Eigen::Vector3d &input, const std::string &output_frame, Eigen::Vector3d *output){
    return convert(input_frame,input,output_frame,output);
}

/**
 * @brief Check if frame conversion is well defined.
 * 
 * @param input_frame Input frame name
 * @param output_frame Output frame name
 * @return true 
 * @return false 
 */
bool GeodeticConverterSynchronized::canConvertSynced(std::string input_frame,std::string output_frame){
    return canConvert(input_frame,output_frame);
}

/**
 * @brief Check if frame is defined
 * 
 * @param frame Name of frame to check
 * @return true 
 * @return false 
 */
bool GeodeticConverterSynchronized::hasFrameSynced(std::string frame){
    return hasFrame(frame);
}

/**
 * @brief Remove a synced frame
 * 
 * @param name Name of frame to remove
 */
void GeodeticConverterSynchronized::removeSyncedFrame(std::string name){
    usv_map::RemoveFrame msg;
    msg.frame_name = name;
    remove_frame_pub_.publish(msg);
}

/**
 * @brief Callback when receiving a message to add a synced EPSG Frame.
 * 
 * @param msg 
 */
void GeodeticConverterSynchronized::EPSGFrameCb(const usv_map::EPSG& msg){
    addFrameByEPSG(msg.frame_name,msg.epsg_code);
}

/**
 * @brief Callback when receiving a message to add a synced ENU frame.
 * 
 * @param msg 
 */
void GeodeticConverterSynchronized::ENUFrameCb(const usv_map::ENU& msg){
    if(hasFrame(msg.frame_name) && !msg.replace){
        ROS_WARN_STREAM("Attempted to add existing frame: " << msg.frame_name);
    } else if(msg.replace){
        removeFrame(msg.frame_name);
    }
    addFrameByENUOrigin(msg.frame_name,msg.lat,msg.lon,msg.alt);
}

/**
 * @brief Callback when receiving a message to remove a frame.
 * 
 * @param msg 
 */
void GeodeticConverterSynchronized::removeFrameCb(const usv_map::RemoveFrame& msg){
    removeFrame(msg.frame_name);
}
}