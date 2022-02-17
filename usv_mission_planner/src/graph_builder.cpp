#include "usv_mission_planner/graph_builder.h"

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
}

Region::Region(double lon_lower, double lat_lower, double width, double height, int depth, int id, int parent_id, childRegion own_region, GDALDataset* ds){
    depth_ = depth;
    id_ = id;
    parent_id_ = parent_id;
    own_region_ = own_region;
    
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
}

int Region::getDepth(){
    return depth_;
}

double Region::getArea(){
    if(region_polygon_==nullptr){
        ROS_ERROR_STREAM("Region::getArea() called with nullptr");
    }
    return region_polygon_->get_Area();
}

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

    #pragma omp parallel for reduction(+:total_area)
        for(auto geom_it = geometries_to_check.begin(); geom_it!=geometries_to_check.end();geom_it++){
            total_area += region_polygon_->Intersection(*geom_it)->toPolygon()->get_Area(); 
            OGRFeature::DestroyFeature(related_features[geom_it-geometries_to_check.begin()]);  
        }
    
    return total_area;
}

Region* Region::getChildRegionContaining(double lon, double lat){
    if (children.size()==0){
        return this;
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

void Region::addChild(Region* child_region_ptr, childRegion child_region){
    /*
    if (region_polygon_!=nullptr){
        //region_polygon_->empty();
        OGRGeometryFactory::destroyGeometry(region_polygon_);
        region_polygon_=nullptr;
    }
    */
    
    children[child_region]=child_region_ptr;
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

int Region::getID(){
    return id_;
}

int Region::getParentID(){
    return parent_id_;
}

childRegion Region::getOwnRegion(){
    return own_region_;
}

Region* Region::getChildRegion(childRegion region_position){
    return children[region_position];
}

Quadtree::Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, bool build_immediately): 
    ds_(ds),
    lower_left_(lower_left),
    upper_right_(upper_right){
    gm_ = new GraphManager;
    ros::Time start = ros::Time::now();
    if (build_immediately) build();
    ros::Time end = ros::Time::now();

    benchmark_data_.build_time = ros::Duration(end-start).toSec();
    std::cout << "Quadtree built, benchmark data: " << std::endl;
    std::cout << "Buildtime total: " << benchmark_data_.build_time << std::endl;
    std::cout << "Regions: " << benchmark_data_.splitRegion_time.size()*4 << std::endl;
    std::cout << "Region build time total: " << std::accumulate(benchmark_data_.splitRegion_time.begin(),benchmark_data_.splitRegion_time.end(),0.0) << std::endl;
    std::cout << "Get occupancy of region total: " << std::accumulate(benchmark_data_.getOccupiedArea_time.begin(),benchmark_data_.getOccupiedArea_time.end(),0.0) << std::endl;
    std::cout << "Size of tree root: " << sizeof(tree_root_) << std::endl;
}

int Quadtree::generateRegionID(){
    return region_id_++;
}

void Quadtree::splitRegion(Region* region, std::queue<Region*>& regions_to_evaluate){
    ros::Time start = ros::Time::now();
    // Calculate NW region
    region->addChild(new Region(region->lower_left_.getX(),region->lower_left_.getY()+region->getHeight()/2,region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::NW,ds_),childRegion::NW);
    //Calculate NE region
    region->addChild(new Region(region->lower_left_.getX()+region->getWidth()/2,region->lower_left_.getY()+region->getHeight()/2,region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::NE,ds_),childRegion::NE);
    //Calculate SW region
    region->addChild(new Region(region->lower_left_.getX(),region->lower_left_.getY(),region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::SW,ds_),childRegion::SW);
    //Calculate SE region
    region->addChild(new Region(region->lower_left_.getX()+region->getWidth()/2,region->lower_left_.getY(),region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::SE,ds_),childRegion::SE);
    //Add child regions for evaluation
    for(std::unordered_map<childRegion,Region *>::iterator it = region->children.begin(); it!=region->children.end(); it++){
        regions_to_evaluate.push((*it).second);
    }
    ros::Time end = ros::Time::now();
    benchmark_data_.splitRegion_time.push_back(ros::Duration(end-start).toSec());
}

std::unordered_map<regionEdge,std::vector<StateVec>> Quadtree::getFramePoints(Region* region){
    std::unordered_map<regionEdge,std::vector<StateVec>> frame_points;
    int divisor = 1;
    //Determine points for south edge and north edge
    int counter = 0;
    for (double x=region->lower_left_.getX(); x<=region->upper_right_.getX();x+=region->getWidth()/divisor){
        counter+=1;
        //South
        frame_points[regionEdge::S].push_back(StateVec(x,region->lower_left_.getY(),0,0));
        //North
        frame_points[regionEdge::N].push_back(StateVec(x,region->upper_right_.getY(),0,0));
    }

    //Determine points for west and east edge
    for (double y=region->lower_left_.getY(); y<=region->upper_right_.getY();y+=region->getHeight()/divisor){
        //West
        frame_points[regionEdge::W].push_back(StateVec(region->lower_left_.getX(),y,0,0));
        //East
        frame_points[regionEdge::E].push_back(StateVec(region->upper_right_.getX(),y,0,0));
    }
    return frame_points;
}

void Quadtree::save(const std::string& tree_name){
    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/quadtrees/"+tree_name+"/");
    //Save graph

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::string graph_path = path+tree_name;
    std::cout << graph_path << std::endl;
    gm_->saveGraph(graph_path);

    //Save for post-mission visualization
    std::string viz_path = path+tree_name+".csv";
    std::ofstream viz_path_file(viz_path);
    viz_path_file << "u_lon,u_lat,v_lon,v_lat,edge_cost\n";
    for(auto edge_it = gm_->edge_map_.begin(); edge_it!=gm_->edge_map_.end();edge_it++){
        Vertex* u = gm_->getVertex((*edge_it).first);
        for (auto vertex_it=(*edge_it).second.begin(); vertex_it!=(*edge_it).second.end(); vertex_it++){
            Vertex* v = gm_->getVertex((*vertex_it).first);
            viz_path_file << u->state.x() << "," << u->state.y() << "," << v->state.x() << "," << v->state.y() << "," << (*vertex_it).second <<"\n";
        }
    }
    viz_path_file.close();

    //Save quadtree
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("SQLite");
    if(driver==NULL){
        ROS_ERROR_STREAM("Unable to find SQLite driver");
        return;
    }

    std::string quadtree_path = path+"quadtree.sqlite";
    GDALDataset* quadtree_ds = driver->Create(quadtree_path.c_str(),0,0,0,GDT_Unknown,NULL);
    if(quadtree_ds==NULL){
        ROS_ERROR_STREAM("Creation of output file failed");
        return;
    }

    std::queue<Region*> regions_to_save;
    regions_to_save.push(tree_root_);
    std::string layer_name_prefix = "depth_";
    std::string layer_name;
    while(regions_to_save.size()!=0){
        Region* current = regions_to_save.front();
        regions_to_save.pop();
        layer_name = layer_name_prefix+std::to_string(current->getDepth());
        OGRLayer* layer = quadtree_ds->GetLayerByName(layer_name.c_str());
        if(layer==NULL){
            layer = quadtree_ds->CreateLayer(layer_name.c_str(),nullptr,wkbPolygon);
            layer->CreateField(new OGRFieldDefn("parent",OGRFieldType::OFTInteger64));
            layer->CreateField(new OGRFieldDefn("id",OGRFieldType::OFTInteger64));
            layer->CreateField(new OGRFieldDefn("region",OGRFieldType::OFTInteger64));
        }
        OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetGeometry(current->region_polygon_);
        feature->SetField("parent",current->getParentID());
        feature->SetField("id",current->getID());
        feature->SetField("region",static_cast<int>(current->getOwnRegion()));
        layer->CreateFeature(feature);

        for(auto child_it = current->children.begin(); child_it!=current->children.end(); child_it++){
            regions_to_save.push((*child_it).second);
        }
        OGRFeature::DestroyFeature(feature);
    }
    ROS_INFO_STREAM("Done saving Quadtree");
}

void Quadtree::load(const std::string& tree_name){
    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/quadtrees/");
    path.append(tree_name+"/");
    std::string graph_path = path + tree_name;
    gm_->loadGraph(graph_path);

    //Load regions
    std::string quadtree_path = path+"quadtree.sqlite";
    ds_ = (GDALDataset*) GDALOpenEx(quadtree_path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    int max_depth = ds_->GetLayerCount()-1;
    int current_depth = 0;
    std::string layer_name;
    std::unordered_map<int,Region*> parent_lookup;
    while(current_depth<=max_depth){
        OGRFeature* feat;
        layer_name = "depth_"+std::to_string(current_depth);
        std::cout << layer_name << std::endl;
        while((feat = ds_->GetLayerByName(layer_name.c_str())->GetNextFeature()) != NULL){
            //Determine parent
            if(feat->GetFieldAsInteger64("id")==feat->GetFieldAsInteger64("parent")){
                //Tree root
                //Determine lower left and upper right
                OGRLineString* boundary = feat->GetGeometryRef()->getBoundary()->toLineString();
                OGRPoint point;
                int num_points = boundary->getNumPoints();
                //Points loded as saved LL->UL->UR->LR->LL (zero-indexed)
                boundary->getPoint(0,&lower_left_);
                boundary->getPoint(2,&upper_right_);

                tree_root_ = new Region(lower_left_,upper_right_,current_depth,generateRegionID(),0,childRegion::NW,ds_);
                
                //Add region to parent_lookup 
                parent_lookup[tree_root_->getID()]=tree_root_;
            } else{
                Region* parent = parent_lookup[feat->GetFieldAsInteger64("parent")];
                switch (static_cast<childRegion>(feat->GetFieldAsInteger64("region")))
                {
                case childRegion::NW:
                    parent->addChild(new Region(parent->lower_left_.getX(),parent->lower_left_.getY()+parent->getHeight()/2,parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,generateRegionID(),parent->getID(),childRegion::NW,ds_),childRegion::NW);
                    break;
                case childRegion::NE:
                    parent->addChild(new Region(parent->lower_left_.getX()+parent->getWidth()/2,parent->lower_left_.getY()+parent->getHeight()/2,parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,generateRegionID(),parent->getID(),childRegion::NE,ds_),childRegion::NE);
                    break;
                case childRegion::SW:
                    parent->addChild(new Region(parent->lower_left_.getX(),parent->lower_left_.getY(),parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,generateRegionID(),parent->getID(),childRegion::SW,ds_),childRegion::SW);
                    break;
                case childRegion::SE:
                    parent->addChild(new Region(parent->lower_left_.getX()+parent->getWidth()/2,parent->lower_left_.getY(),parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,generateRegionID(),parent->getID(),childRegion::SE,ds_),childRegion::SE);
                    break;
                default:
                    ROS_ERROR_STREAM("Region type not recognized!");
                    break;
                }
                //Add child to parent map
                parent_lookup[parent->getChildRegion(static_cast<childRegion>(feat->GetFieldAsInteger64("region")))->getID()]=parent->getChildRegion(static_cast<childRegion>(feat->GetFieldAsInteger64("region")));

                if (parent->children.size()==4){
                    //All 4 children added, remove from map
                    parent_lookup.erase(parent->getID());
                }
            }
        }
        OGRFeature::DestroyFeature(feat);
        current_depth++;   


    }
    
}

Region* Quadtree::getLeafRegionContaining(double lon, double lat){
    Region* current = tree_root_;
    Region* prev = nullptr;
    while(current!=prev){
        region_sequence_.push_back(current);
        prev = current;
        current = current->getChildRegionContaining(lon,lat);
    }
    return current;
}

void Quadtree::build(){
    std::queue<Region*> regions_to_evaluate;
    tree_root_ = new Region(lower_left_,upper_right_,0,generateRegionID(),0,childRegion::NW,ds_);
    regions_to_evaluate.push(tree_root_);
    while(!regions_to_evaluate.empty()){
        Region* current_region = regions_to_evaluate.front();
        regions_to_evaluate.pop();
        if(current_region->getArea()<pow(10,-7)){
            continue;
        }

        ros::Time start = ros::Time::now();
        double occupied_ratio = current_region->getOccupiedRatio();
        ros::Time end = ros::Time::now();
        benchmark_data_.getOccupiedArea_time.push_back(ros::Duration(end-start).toSec());
        
        if (occupied_ratio==1.0){
            //Region is definitely occupied, discard it
            continue;
        } else if(occupied_ratio<0.001 ){
            //Area is free, add vertecies
            std::unordered_map<regionEdge,std::vector<StateVec>> frame_points = getFramePoints(current_region);
            Vertex* corner;
            for (auto side_it = frame_points.begin(); side_it!=frame_points.end(); side_it++){
                for (auto state_it = (*side_it).second.begin(); state_it != (*side_it).second.end(); state_it++){
                    if (!gm_->getNearestVertexInRange(&(*state_it),0.0001,&corner)){
                        corner = new Vertex(gm_->generateVertexID(),*state_it);
                        current_region->vertices.push_back(corner);
                        gm_->addVertex(corner);
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
                }
            }
        } else{
            //The region is not definitely occupied definitely free, split it!
            splitRegion(current_region,regions_to_evaluate);
        }

    }

}

QuadtreeROS::QuadtreeROS(ros::NodeHandle& nh, OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds,bool build_immediately):
nh_(nh),
Quadtree(lower_left,upper_right,ds,build_immediately){
    vertex_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/quadtree/visual_vertices",1,true);
    edge_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/quadtree/visual_edges",1,true);
    region_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/quadtree/highlight_region",1,true);
    test_point_pub_ = nh_.advertise<visualization_msgs::Marker>("/quadtree/test_point",1,true);

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

void QuadtreeROS::testGetRegion(double lon, double lat){

    Eigen::Vector3d point_geo(lon,lat,0);
    Eigen::Vector3d point_enu;
    geo_converter_.convert("WGS84",point_geo,"global_enu",&point_enu);
    test_point_.pose.position.x = point_enu.x();
    test_point_.pose.position.y = point_enu.y();
    test_point_pub_.publish(test_point_);

    ros::Time start = ros::Time::now();
    Region* child = getLeafRegionContaining(lon,lat);
    ros::Time end = ros::Time::now();
    std::cout << "Finding child leaf took: " << ros::Duration(end-start).toSec() << std::endl;


    highlightRegion(child);   
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
    
    vertex_marker_.color.r = 1.0f;
    vertex_marker_.color.g = 0.0f;
    vertex_marker_.color.b = 0.0f;
    vertex_marker_.color.a = 0.5;

    // Scale unit is meters
    vertex_marker_.scale.x = 0.35;
    vertex_marker_.scale.y = 0.35;
    vertex_marker_.scale.z = 0.35;

    vertex_marker_.pose.orientation.w = 1.0;
    vertex_marker_.id = 0;

    edge_marker_ = vertex_marker_;
    edge_marker_.type = visualization_msgs::Marker::LINE_LIST;

    region_marker_ = edge_marker_;
    region_marker_.color.b = 0.0f;
    region_marker_.color.g = 1.0f;
    region_marker_.scale.x = 1;
    region_marker_.scale.y = 1;
    region_marker_.scale.z = 1;

    test_point_ = edge_marker_;
    test_point_.type = visualization_msgs::Marker::SPHERE;
    test_point_.scale.x = 10;
    test_point_.scale.y = 10;
    test_point_.scale.z = 10;
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

void QuadtreeROS::highlightRegion(Region* region){
    region_marker_.points.clear();
    double lon_SW = region->lower_left_.getX();
    double lat_SW = region->lower_left_.getY();

    double lon_NW = region->lower_left_.getX();
    double lat_NW = region->upper_right_.getY();

    double lon_SE = region->upper_right_.getX();
    double lat_SE = region->lower_left_.getY();

    double lon_NE = region->upper_right_.getX();
    double lat_NE = region->upper_right_.getY();
    std::cout << "SW: " << lon_SW << " " << lat_SW << std::endl;
    std::cout << "NW: " << lon_NW << " " << lat_NW << std::endl;
    std::cout << "SE: " << lon_SE << " " << lat_SE << std::endl;
    std::cout << "NE: " << lon_NE << " " << lat_NE << std::endl;

    Eigen::Vector3d SW_geo(lon_SW,lat_SW,0);
    Eigen::Vector3d NW_geo(lon_NW,lat_NW,0);
    Eigen::Vector3d SE_geo(lon_SE,lat_SE,0);
    Eigen::Vector3d NE_geo(lon_NE,lat_NE,0);

    Eigen::Vector3d SW_enu;
    Eigen::Vector3d NW_enu;
    Eigen::Vector3d SE_enu;
    Eigen::Vector3d NE_enu;

    geo_converter_.convert("WGS84",SW_geo,"global_enu",&SW_enu);
    geo_converter_.convert("WGS84",NW_geo,"global_enu",&NW_enu);
    geo_converter_.convert("WGS84",SE_geo,"global_enu",&SE_enu);
    geo_converter_.convert("WGS84",NE_geo,"global_enu",&NE_enu);

    geometry_msgs::Point point;
    point.x = SW_enu.x();
    point.y = SW_enu.y();
    region_marker_.points.push_back(point);

    point.x = NW_enu.x();
    point.y = NW_enu.y();
    region_marker_.points.push_back(point);
    region_marker_.points.push_back(point);

    point.x = NE_enu.x();
    point.y = NE_enu.y();
    region_marker_.points.push_back(point);
    region_marker_.points.push_back(point);

    point.x = SE_enu.x();
    point.y = SE_enu.y();
    region_marker_.points.push_back(point);
    region_marker_.points.push_back(point);

    point.x = SW_enu.x();
    point.y = SW_enu.y();
    region_marker_.points.push_back(point);

    region_marker_pub_.publish(region_marker_);
}





