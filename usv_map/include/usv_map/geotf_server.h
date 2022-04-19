#include "usv_map/geotf_sync.h"
#include "usv_map/frame_conversion.h"
#include "usv_map/add_frame.h"


namespace geotf {

class GeodeticConverterServer : private GeodeticConverterSynchronized {
    public:
        GeodeticConverterServer(const ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer frame_conversion_srv_;
        ros::ServiceServer add_frame_srv_;

        bool frameConversion(usv_map::frame_conversion::Request& req, usv_map::frame_conversion::Response& res);
        bool addFrame(usv_map::add_frame::Request& req, usv_map::add_frame::Response& res);
        //Utilities
};

Eigen::Vector3d pointToVector(geometry_msgs::Point& point);
geometry_msgs::Point vectorToPoint(Eigen::Vector3d& vec);

}//End geotf namespace