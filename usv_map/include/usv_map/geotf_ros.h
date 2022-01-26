#include "geotf/geodetic_converter.h"
#include "usv_map/frame_conversion.h"
#include "usv_map/add_frame.h"

namespace geotf {

class GeodeticConverterServer : GeodeticConverter {
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

class GeodeticConverterClient {
    public:
        GeodeticConverterClient(ros::NodeHandle* nh);
        void convert(std::string& from_frame, Eigen::Vector3d& from_point, std::string& to_frame, Eigen::Vector3d& to_point);
        void addFrame(std::string name, double lon, double lat, double alt);
    private:
        ros::NodeHandle* nh_;
        ros::ServiceClient add_frame_client_;
        ros::ServiceClient convert_point_client_;



};

Eigen::Vector3d pointToVector(geometry_msgs::Point& point);
geometry_msgs::Point vectorToPoint(Eigen::Vector3d& vec);

}//End geotf namespace