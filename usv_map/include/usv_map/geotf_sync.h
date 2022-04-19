#pragma once

#include "geotf/geodetic_converter.h"
#include "ros/ros.h"
#include "usv_map/EPSG.h"
#include "usv_map/ENU.h"
#include "usv_map/RemoveFrame.h"

namespace geotf{

class GeodeticConverterSynchronized : private GeodeticConverter{
    public:
        GeodeticConverterSynchronized(const ros::NodeHandle& nh);
        void addSyncedFrameByEPSG(std::string name, int code);
        void addSyncedFrameByENUOrigin(std::string name, double lon, double lat, double alt);
        void removeSyncedFrame(std::string name);
        bool convertSynced(std::string input_frame, const Eigen::Vector3d &input, const std::string &output_frame, Eigen::Vector3d *output);
        bool canConvertSynced(std::string input_frame, std::string output_frame);
        bool hasFrameSynced(std::string frame);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber epsg_frame_sub_;
        ros::Subscriber enu_frame_sub_;
        ros::Subscriber remove_frame_sub_;
        ros::Publisher epsg_frame_pub_;
        ros::Publisher enu_frame_pub_;
        ros::Publisher remove_frame_pub_;

        void EPSGFrameCb(const usv_map::EPSG& msg);
        void ENUFrameCb(const usv_map::ENU& msg);
        void removeFrameCb(const usv_map::RemoveFrame& msg);
};


}//Namespace geotf
