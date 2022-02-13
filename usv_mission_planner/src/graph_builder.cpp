#include "usv_mission_planner/graph_builder.h"

Region::Region(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds):
    ds_(ds),
    lower_left_(lower_left),
    upper_right_(upper_right){
    OGRLinearRing tmp;
    tmp.addPoint(&lower_left);
    tmp.addPoint(lower_left.getX(),upper_right.getY());
    tmp.addPoint(&upper_right);
    tmp.addPoint(upper_right.getX(),lower_left.getY());
    tmp.addPoint(&lower_left);

    region_polygon_.addRingDirectly(new OGRLinearRing(&tmp));

    if(!region_polygon_.IsValid())
    {
        ROS_ERROR_STREAM("Region polygon not valid");
    }

    comparison_layer_ = ds->GetLayerByName("collision_dissolved");
}

Region::Region(double lon_lower, double lat_lower, double width, double height, GDALDataset* ds){
    OGRPoint lower_left;
    lower_left.setX(lon_lower);
    lower_left.setY(lat_lower);

    OGRPoint upper_right;
    upper_right.setX(lon_lower+width);
    upper_right.setY(lat_lower+height);

    lower_left_ = lower_left;
    upper_right_= upper_right;

    OGRLinearRing tmp;
    tmp.addPoint(&lower_left);
    tmp.addPoint(lower_left.getX(),upper_right.getY());
    tmp.addPoint(&upper_right);
    tmp.addPoint(upper_right.getX(),lower_left.getY());
    tmp.addPoint(&lower_left);

    region_polygon_.addRingDirectly(new OGRLinearRing(&tmp));

    if(!region_polygon_.IsValid())
    {
        ROS_ERROR_STREAM("Region polygon not valid");
    }

    comparison_layer_ = ds->GetLayerByName("collision_dissolved");
}

double Region::getArea(){
    return region_polygon_.get_Area();
}

double Region::getOccupiedArea(){
    OGRFeature* feat;
    double total_area;
    while((feat = comparison_layer_->GetNextFeature()) != NULL){
        OGRGeometry *geom = feat->GetGeometryRef();
        total_area += region_polygon_.Intersection(geom)->toPolygon()->get_Area();
    }
    return total_area;
}

Region* Region::getChildRegionContaining(double lon, double lat){
    //TODO
    return nullptr;
}

void Region::addChild(Region* child_region_ptr){
    children.push_back(child_region_ptr);
}

double Region::getWidth(){
    return upper_right_.getX()-lower_left_.getX();
}

double Region::getHeight(){
    return upper_right_.getY()-lower_left_.getY();
}

double Region::getOccupiedRatio(){
    double total_area = getArea();
    double occupied_area = getOccupiedArea();
    return occupied_area/total_area;
}

Quadtree::Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds): 
    ds_(ds),
    lower_left_(lower_left),
    upper_right_(upper_right){
    gm_ = new GraphManager;

    std::cout << "Start building quadtree" << std::endl;
    build();
    std::cout << "Quadtree built" << std::endl;
}

void Quadtree::splitRegion(Region* region, std::queue<Region*>& regions_to_evaluate){
    // Calculate NW region
    region->addChild(new Region(region->lower_left_.getX(),region->lower_left_.getY()+region->getHeight()/2,region->getWidth()/2,region->getHeight()/2,ds_));
    //Calculate NE region
    region->addChild(new Region(region->lower_left_.getX()+region->getWidth()/2,region->lower_left_.getY()+region->getHeight()/2,region->getWidth()/2,region->getHeight()/2,ds_));
    //Calculate SW region
    region->addChild(new Region(region->lower_left_.getX(),region->lower_left_.getY(),region->getWidth()/2,region->getHeight()/2,ds_));
    //Calculate SE region
    region->addChild(new Region(region->lower_left_.getX()+region->getWidth()/2,region->lower_left_.getY(),region->getWidth()/2,region->getHeight()/2,ds_));
    //Add child regions for evaluation
    for(std::vector<Region *>::iterator it = region->children.begin(); it!=region->children.end(); it++){
        regions_to_evaluate.push(*it);
    }
}

std::unordered_map<regionEdge,std::vector<StateVec>> Quadtree::getFramePoints(Region* region){
    std::unordered_map<regionEdge,std::vector<StateVec>> frame_points;
    int divisor = 4;
    //Determine points for south edge and north edge
    for (float x=region->lower_left_.getX(); x<=region->upper_right_.getX();x+=region->getWidth()/divisor){
        //South
        frame_points[regionEdge::S].push_back(StateVec(x,region->lower_left_.getY(),0,0));
        //North
        frame_points[regionEdge::N].push_back(StateVec(x,region->upper_right_.getY(),0,0));
    }

    //Determine points for west and east edge
    for (float y=region->lower_left_.getY(); y<=region->upper_right_.getY();y+=region->getHeight()/divisor){
        //West
        frame_points[regionEdge::W].push_back(StateVec(region->lower_left_.getX(),y,0,0));
        //East
        frame_points[regionEdge::E].push_back(StateVec(region->upper_right_.getX(),y,0,0));
    }
    return frame_points;
}

void Quadtree::build(){
    std::queue<Region*> regions_to_evaluate;
    tree_root_ = new Region(lower_left_,upper_right_,ds_);
    regions_to_evaluate.push(tree_root_);
    while(!regions_to_evaluate.empty()){
        Region* current_region = regions_to_evaluate.front();
        regions_to_evaluate.pop();

        double occupied_ratio = current_region->getOccupiedRatio();
        if (occupied_ratio>0.95 || current_region->getArea()<pow(10,-7)){
            //Region is definitely occupied, discard it
            continue;
        } else if(occupied_ratio<0.005 ){
            //Area is free, add vertecies
            std::unordered_map<regionEdge,std::vector<StateVec>> frame_points = getFramePoints(current_region);
            Vertex* corner;
            for (auto side_it = frame_points.begin(); side_it!=frame_points.end(); side_it++){
                for (auto state_it = (*side_it).second.begin(); state_it != (*side_it).second.end(); state_it++){
                    if (!gm_->getNearestVertexInRange(&(*state_it),0.0001,&corner)){
                        corner = new Vertex(gm_->generateVertexID(),*state_it);
                        current_region->vertices.push_back(corner);
                        gm_->addVertex(corner);
                        //addVisualVertex(corner);
                    } else{
                        current_region->vertices.push_back(corner);
                    }
                }
            }

            //Add edges
            for(auto it_vert_a=current_region->vertices.begin();it_vert_a!=current_region->vertices.end();it_vert_a++){
                for(auto it_vert_b=current_region->vertices.begin();it_vert_b!=current_region->vertices.end();it_vert_b++){
                    if (*it_vert_a==*it_vert_b) continue;

                    //If edge already exists, don't add again
                    if(gm_->edge_map_[(*it_vert_a)->id].find((*it_vert_b)->id)!=gm_->edge_map_[(*it_vert_a)->id].end()) continue;

                    //TODO: Wrong to use Euler here to weight the edge, must uuse geodetic distance!
                    float distance = sqrt(pow((*it_vert_a)->state.x()-(*it_vert_b)->state.x(),2)+pow((*it_vert_a)->state.y()-(*it_vert_b)->state.y(),2));
                    gm_->addEdge(*it_vert_a,*it_vert_b,distance);
                    //addVisualEdge(*it_vert_a,*it_vert_b);
                }
            }
        } else{
            //The region is not definitely occupied definitely free, split it!
            splitRegion(current_region,regions_to_evaluate);
        }

    }

}

QuadtreeROS::QuadtreeROS(ros::NodeHandle& nh, OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds):
nh_(nh),
Quadtree(lower_left,upper_right,ds){
    vertex_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/quadtree/visual_vertices",1,true);
    edge_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/quadtree/visual_edges",1,true);

    geo_converter_.addFrameByEPSG("WGS84",4326);
    geo_converter_.addFrameByENUOrigin("global_enu",40.5612,-73.9761,0);

    initializeMarkers();
}

void QuadtreeROS::visualize(){
    for(auto edge_it = gm_->edge_map_.begin(); edge_it!=gm_->edge_map_.end();edge_it++){
        Vertex* u = gm_->getVertex((*edge_it).first);
        Eigen::Vector3d u_wgs_eig(u->state.x(),u->state.y(),0);
        Eigen::Vector3d u_enu_eig;
        geo_converter_.convert("WGS84",u_wgs_eig,"global_enu",&u_enu_eig);
        addVisualVertex(u_enu_eig);
        for (auto vertex_it=(*edge_it).second.begin(); vertex_it!=(*edge_it).second.end(); vertex_it++){
            Vertex* v = gm_->getVertex((*vertex_it).first);
            //Convert to flat ENU projection
            Eigen::Vector3d v_wgs_eig(v->state.x(),v->state.y(),0);
            Eigen::Vector3d v_enu_eig;
            geo_converter_.convert("WGS84",v_wgs_eig,"global_enu",&v_enu_eig);
            addVisualVertex(v_enu_eig);
            addVisualEdge(u_enu_eig,v_enu_eig);
        }
    }
    publishVisualGraph();
}

void QuadtreeROS::initializeMarkers(){
    vertex_marker_.header.frame_id = "map";
    vertex_marker_.header.stamp = ros::Time::now();
    vertex_marker_.type = visualization_msgs::Marker::CUBE_LIST;
    vertex_marker_.action = visualization_msgs::Marker::ADD;
    vertex_marker_.lifetime = ros::Duration();

    vertex_marker_.pose.position.x = 0.0;
    vertex_marker_.pose.position.y = 0.0;
    vertex_marker_.pose.position.z = 0.0;
    
    vertex_marker_.color.r = 0.0f;
    vertex_marker_.color.g = 0.0f;
    vertex_marker_.color.b = 1.0f;
    vertex_marker_.color.a = 1.0;

    // Scale unit is meters
    vertex_marker_.scale.x = 0.25;
    vertex_marker_.scale.y = 0.25;
    vertex_marker_.scale.z = 0.25;

    vertex_marker_.pose.orientation.w = 1.0;
    vertex_marker_.id = 0;

    edge_marker_ = vertex_marker_;
    edge_marker_.type = visualization_msgs::Marker::LINE_LIST;
}

void QuadtreeROS::addVisualVertex(Eigen::Vector3d& vertex) {
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;

    point.x = vertex.x();
    point.y = vertex.y();
    vertex_marker_.points.push_back(point);
    vertex_marker_.colors.push_back(color);
}

void QuadtreeROS::addVisualEdge(Eigen::Vector3d& from_vertex, Eigen::Vector3d& to_vertex){
    geometry_msgs::Point point;
    point.x = from_vertex.x();
    point.y = from_vertex.y();
    edge_marker_.points.push_back(point);

    point.x = to_vertex.x();
    point.y = to_vertex.y();
    edge_marker_.points.push_back(point);
}






