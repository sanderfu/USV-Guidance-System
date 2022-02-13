#pragma once

#include "ros/ros.h"
#include "ros/package.h"
#include "gdal/ogrsf_frmts.h"
#include "planner_common/graph_manager.h"
#include "vector"
#include "queue"
#include <unordered_map>

#include "visualization_msgs/Marker.h"
#include "geotf/geodetic_converter.h"

enum regionEdge{
    N,S,E,W
};

class Region{
    public:
        Region(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds);
        Region(double lon_lower, double lat_lower, double lon_upper, double lat_upper, GDALDataset* ds);
        Region* getChildRegionContaining(double lon, double lat);
        void addChild(Region* child_region_ptr);

        double getWidth();
        double getHeight();
        double getOccupiedRatio();
        double getOccupiedArea();
        double getArea();

        OGRPoint lower_left_;
        OGRPoint upper_right_;

        std::vector<Region*> children;
        std::vector<Vertex*> vertices;
    private:
        GDALDataset* ds_;
        OGRLayer* comparison_layer_;
        OGRPolygon region_polygon_;
};

class Quadtree{
    public:
        Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, bool build_immediately=true);
        void setStart(Vertex* s);
        void setGoal(Vertex* g);

        void save();
        void load(const std::string& tree_name);

    protected:
        GDALDataset* ds_;
        GraphManager* gm_;
        Region* tree_root_;

        OGRPoint lower_left_;
        OGRPoint upper_right_;

        std::unordered_map<regionEdge,std::vector<StateVec>> getFramePoints(Region* region);
        void build();

        void splitRegion(Region* region, std::queue<Region*>& regions_to_evaluate);
        Region* findLeafRegionContaining(StateVec& pos);
        void setCustomVertex(Vertex* s);
};

class QuadtreeROS : public Quadtree{
    public:
        QuadtreeROS(ros::NodeHandle& nh, OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, bool build_immediately=true);
        void visualize();
    private:
        geotf::GeodeticConverter geo_converter_;
        ros::NodeHandle nh_;
        ros::Publisher vertex_marker_pub_;
        ros::Publisher edge_marker_pub_;

        visualization_msgs::Marker edge_marker_;
        visualization_msgs::Marker vertex_marker_;

        //Visualization
        void initializeMarkers();
        void addVisualVertex(Eigen::Vector3d& vertex);
        void addVisualEdge(Eigen::Vector3d& from_vertex, Eigen::Vector3d& to_vertex);
        
        inline void publishVisualGraph(){
            std::cout << "Edge marker frame id:" << edge_marker_.header.frame_id << std::endl;
            vertex_marker_pub_.publish(vertex_marker_);
            edge_marker_pub_.publish(edge_marker_);
        }
};
