#include "usv_map/region.h"

/**
 * @brief Construct a new Region object.
 * 
 * @remark Only used to construct tree root
 * 
 * @param lower_left The lower left point defining the region.
 * @param upper_right The upper right point defining the region
 * @param depth How many times regions have been split to get to this region. Tree root has depth 0.
 * @param id A unique ID for the region.
 * @param parent_id The unique ID of the parent region.
 * @param own_region A description of the regions position w.r.t. to the parent region centorid.
 * @param ds The datasource from which a map to counstruct a quadtree around is located.
 */
Region::Region(OGRPoint lower_left, OGRPoint upper_right, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds):
    ds_(ds),
    lower_left_(lower_left),
    upper_right_(upper_right),
    depth_(depth),
    id_(id),
    parent_id_(parent_id){
    OGRLinearRing tmp;
    tmp.addPoint(&lower_left);
    tmp.addPoint(lower_left.getX(),upper_right.getY());
    tmp.addPoint(&upper_right);
    tmp.addPoint(upper_right.getX(),lower_left.getY());
    tmp.addPoint(&lower_left);

    region_polygon_ = new OGRPolygon;
    region_polygon_->addRingDirectly(new OGRLinearRing(&tmp));

    if(!region_polygon_->IsValid())
    {
        ROS_ERROR_STREAM("Region polygon not valid");
    }

    //Store region center
    region_polygon_->Centroid(&centroid_);

    comparison_layer_ = ds->GetLayerByName("collision_dissolved");
    unknown_layer_ = ds->GetLayerByName("unknown");
    
}

/**
 * @brief Construct a new Region object
 * 
 * @param lon_lower Longitude of the lower left point defining the region
 * @param lat_lower Latitude of the lower left point defining the region
 * @param width Width of the region [deg]
 * @param height Height of the region [deg]
 * @param depth How many times regions have been split to get to this region. Tree root has depth 0.
 * @param id A unique ID for the region.
 * @param parent_id The unique ID of the parent region.
 * @param own_region A description of the regions position w.r.t. to the parent region centorid.
 * @param ds The datasource from which a map to counstruct a quadtree around is located.
 */
Region::Region(double lon_lower, double lat_lower, double width, double height, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds){
    depth_ = depth;
    id_ = id;
    parent_id_ = parent_id;
    own_region_ = own_region;
    is_leaf_ = false;
    
    lower_left_.setX(lon_lower);
    lower_left_.setY(lat_lower);

    upper_right_.setX(lon_lower+width);
    upper_right_.setY(lat_lower+height);

    OGRLinearRing tmp;
    tmp.addPoint(lon_lower,lat_lower);
    tmp.addPoint(lon_lower,lat_lower+height);
    tmp.addPoint(lon_lower+width,lat_lower+height);
    tmp.addPoint(lon_lower+width,lat_lower);
    tmp.addPoint(lon_lower,lat_lower);

    region_polygon_ = new OGRPolygon;
    region_polygon_->addRingDirectly(new OGRLinearRing(&tmp));

    if(!region_polygon_->IsValid())
    {
        ROS_ERROR_STREAM("Region polygon not valid");
    }

    //Store region center
    region_polygon_->Centroid(&centroid_);

    comparison_layer_ = ds->GetLayerByName("collision_dissolved");
    unknown_layer_ = ds->GetLayerByName("unknown");
}

/**
 * @brief Get the unique ID of the region
 * 
 * @return int ID
 */
int Region::getID(){
    return id_;
}

/**
 * @brief Get depth of region in quadtree
 * 
 * @return int Region depth
 */
int Region::getDepth(){
    return depth_;
}

/**
 * @brief Get the unique ID of region parent
 * 
 * @return int Parent ID
 */
int Region::getParentID(){
    return parent_id_;
}

/**
 * @brief Get region relative position to parent centroid.
 * 
 * @return childRegion Relative position
 */
childRegion Region::getOwnRegion(){
    return own_region_;
}


/**
 * @brief Get region width
 * 
 * @return double Region width [deg]
 */
double Region::getWidth(){
    return upper_right_.getX()-lower_left_.getX();
}

/**
 * @brief Get region height
 * 
 * @return double Height [deg]
 */
double Region::getHeight(){
    return upper_right_.getY()-lower_left_.getY();
}

/**
 * @brief Get the area of the region.
 * 
 * @remark If the region polygon is not set, the impossible
 * value -1 is returned.
 * 
 * @return double The region area [degrees squared]
 */
double Region::getArea(){
    if(region_polygon_==nullptr){
        ROS_ERROR_STREAM("Region::getArea() called with nullptr");
        return -1;
    }
    return region_polygon_->get_Area();
}

/**
 * @brief Get the total area that is occupied in the region by some defined layer in ds_
 * 
 * @remark This is the most costly function when building the quadtree due to the intersection procedure called.
 * 
 * @return double The occupied area [degrees squared]
 */
double Region::getOccupiedArea(){

    if(region_polygon_==nullptr){
        ROS_ERROR_STREAM("Region::getOccupiedArea() called with nullptr");
    }

    OGRFeature* feat;
    double total_area = 0;

    std::vector<OGRGeometry*> geometries_to_check;
    std::vector<OGRFeature*> related_features;

    while((feat = comparison_layer_->GetNextFeature()) != NULL){
        geometries_to_check.push_back(feat->GetGeometryRef()); 
        related_features.push_back(feat);
    }

    while((feat = unknown_layer_->GetNextFeature()) != NULL){
        geometries_to_check.push_back(feat->GetGeometryRef()); 
        related_features.push_back(feat);
    }



    #pragma omp parallel for reduction(+:total_area)
        for(auto geom_it = geometries_to_check.begin(); geom_it!=geometries_to_check.end();geom_it++){
            total_area += region_polygon_->Intersection(*geom_it)->toPolygon()->get_Area(); 
            OGRFeature::DestroyFeature(related_features[geom_it-geometries_to_check.begin()]);  
        }
    
    return total_area;
}

/**
 * @brief Get occupied area ratio
 * 
 * @return double Occupied relative to total ratio
 */
double Region::getOccupiedRatio(){
    double total_area = getArea();
    double occupied_area = getOccupiedArea();
    return occupied_area/total_area;
}

/**
 * @brief Given a relative position to the centroid of this region, get child pointer.
 * 
 * @remark If region is quadtree leaf, nullptr is returned.
 * 
 * @param region_position Relative position of the child.
 * @return Region* The child pointer
 */
Region* Region::getChildRegion(childRegion region_position){
    if(children.size()==0) return nullptr;
    return children[region_position];
}


/**
 * @brief Retrieve a pointer to the child region of the given parent region containing the defined point.
 * 
 * @param lon Point longitude
 * @param lat Point latitude
 * @return Region* Pointer to the child region
 */
Region* Region::getChildRegionContaining(double lon, double lat){
    if (children.size()==0){
        if(is_leaf_ && lon>=lower_left_.getX() && lon<=upper_right_.getX() && lat>=lower_left_.getY() && lat<=upper_right_.getY()) return this;
        else return nullptr;
    }
    if (lon<=centroid_.getX()){
        //Left side
        if (lat<=centroid_.getY()){
            //LL
            return children[childRegion::SW];
        } else{
            //UL
            return children[childRegion::NW];
        }
    }else{
        //Right side
        if(lat<=centroid_.getY()){
            //LR
            return children[childRegion::SE];
        } else{
            //UR
            return children[childRegion::NE];
        }
    }

}

/**
 * @brief Add a child region to a region.
 * 
 * @remark Thus function should be called such that the child region is created on the Heap
 * with pointer stored by the parent only.
 * 
 * @param child_region_ptr The child region
 * @param child_region The child region position relative to parent centroid.
 */
void Region::addChild(Region* child_region_ptr, childRegion child_region){
    children.insert(std::make_pair(child_region,child_region_ptr));
}