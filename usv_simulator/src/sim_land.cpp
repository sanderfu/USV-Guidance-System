#include "usv_simulator/sim_land.h"
/**
 * @brief Construct a new SimulatedLand object.
 * @details Advertise the relevant topic, register the 
 * frame converter and load the polygons. 
 * 
 * @param nh ROS NodeHandle
 */
SimulatedLand::SimulatedLand(const ros::NodeHandle& nh){
    std::string map_name;
    bool parameter_load_error = false;
    if(!ros::param::get("map_name",map_name)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
    ros::topic::waitForMessage<std_msgs::Bool>("mission_planner/region_available");
    path_ = ros::package::getPath("usv_map")+"/data/mission_regions/"+map_name+"/region.sqlite";
    
    std::vector<double> global_position_vec;
    if(!nh_.getParam("sim_origin",global_position_vec)){
        ROS_ERROR_STREAM("Failed to load initial position parameter");
    }
    polygon_.header.frame_id="map";
    poly_pub_ = nh_.advertise<usv_simulator::PolygonArray64>("/sim/land",1,true);
    converter_.addFrameByENUOrigin("local",global_position_vec[1],global_position_vec[0],0);
    converter_.addFrameByEPSG("global",4326);
    if(!converter_.canConvert("global","local")){
        ROS_ERROR_STREAM_NAMED("sim_land","Frame conversion error");
        ros::shutdown();
    }
    loadPolygons();
}

/**
 * @brief Load landmass from sqlite database and publish to RVIZ. 
 * For sim visualization and vessel debugging purposes only
 * 
 */
void SimulatedLand::loadPolygons(){
    GDALAllRegister();
    GDALDataset *ds_ptr_;
    ds_ptr_ = (GDALDataset*) GDALOpenEx(path_.c_str(),GDAL_OF_VECTOR,NULL,NULL,NULL);
    if (ds_ptr_==NULL){
        ROS_ERROR_STREAM_NAMED("sim_land","Failed to open map");
        ros::shutdown();
    }
    OGRLayer* collision_layer = ds_ptr_->GetLayerByName("collision_dissolved");
    OGRFeature* feat = collision_layer->GetNextFeature();
    collision_layer->ResetReading();
    while((feat=collision_layer->GetNextFeature())!=NULL){
        polygon_.points.clear();
        OGRLinearRing* ext_ring = feat->GetGeometryRef()->toPolygon()->getExteriorRing();
        for(int i=0;i<ext_ring->getNumPoints()-1;i++){
            Eigen::Vector3d global_coord(ext_ring->getX(i),ext_ring->getY(i),0);
            Eigen::Vector3d local_coord;
            converter_.convert("global",global_coord,"local",&local_coord);
            point_.x = local_coord(0);
            point_.y = local_coord(1);
            polygon_.points.push_back(point_);
        }
        if(ext_ring->getNumPoints()>1){
            polygon_array_.polygons.push_back(polygon_);
        }
        OGRFeature::DestroyFeature(feat);
    }

    polygon_array_.header.frame_id="map";
    poly_pub_.publish(polygon_array_);
}