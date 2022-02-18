#include "usv_mission_planner/quadtree.h"

/**
 * @brief Construct a new Quadtree object
 * 
 * @param lower_left The lower left point defining the region in which the quadtree is to be built.
 * @param upper_right The upper right point defining the region in which the quadtree is to be built.
 * @param ds The dataset containing the map we build the Quadtree w.r.t. 
 * @param build_immediately If true, build the quadtree. If false, dont.
 */
Quadtree::Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, bool build_immediately): 
    ds_(ds),
    lower_left_(lower_left),
    upper_right_(upper_right),
    geod_(GeographicLib::Geodesic::WGS84()){
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

/**
 * @brief Save the quadtree for later loading or post-mission visualization.
 * 
 * @param tree_name What the quadtree should be saved as (will overwrite any previous stored tree witht his name)
 */
void Quadtree::save(const std::string& tree_name){
    std::string path = ros::package::getPath("usv_mission_planner");
    path.append("/data/quadtrees/"+tree_name+"/");
    //Save graph

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::string graph_path = path+tree_name;
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
    GDALDriver* driver_memory = GetGDALDriverManager()->GetDriverByName("Memory");
    GDALDriver* driver_sqlite = GetGDALDriverManager()->GetDriverByName("SQLite");
    if(driver_memory==NULL){
        ROS_ERROR_STREAM("Unable to find Memory driver");
        return;
    }
    if(driver_sqlite==NULL){
        ROS_ERROR_STREAM("Unable to find SQLite driver");
        return;
    }

    std::string quadtree_path = path+"quadtree.sqlite";
    GDALDataset* quadtree_ds = driver_memory->Create("in_mem",0,0,0,GDT_Unknown,NULL);
    if(quadtree_ds==NULL){
        ROS_ERROR_STREAM("Creation of in-memory dataset failed");
        return;
    }

    std::queue<Region*> regions_to_save;
    regions_to_save.push(tree_root_);
    std::string layer_name_prefix = "depth_";
    std::string layer_name;
    OGRLayer* test_layer;
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
    GDALDataset* quadtree_ds_store = driver_sqlite->Create(quadtree_path.c_str(),0,0,0,GDT_Unknown,NULL);
    if(quadtree_ds_store==NULL){
        ROS_ERROR_STREAM("Creation of SQLite database failed");
        return;
    }

    for (int i=0; i<quadtree_ds->GetLayerCount(); i++){
        quadtree_ds_store->CopyLayer(quadtree_ds->GetLayer(i),quadtree_ds->GetLayer(i)->GetName());
    }
    ROS_INFO_STREAM("Quadtree saved succesfully");
}

/**
 * @brief Load a saved quadtree.
 * 
 * @param tree_name Nam eof the saved quadtree. If a non-existent quadtree is named, the code will crash hard!
 */
void Quadtree::load(const std::string& tree_name){
    std::string path = ros::package::getPath("usv_mission_planner")+"/data/quadtrees/"+tree_name+"/";
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
    ROS_INFO_STREAM("Quadtree loaded succesfully");
}

/**
 * @brief For a given point, find the leaf region containing the point.
 * 
 * @param lon Point longitude
 * @param lat Point latitude
 * @return Region* Leaf region pointer
 */
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

GraphManager* Quadtree::getGraphManager(){
    return gm_;
}

/**
 * @brief Generate a unique region ID
 * 
 * @return int generated ID
 */
int Quadtree::generateRegionID(){
    return region_id_++;
}

/**
 * @brief Split a region into four children and add these children to the evaluation queue.
 * 
 * @details The child is created on the Heap with pointer stored by the parent.
 * 
 * @param region Parent region to split into four quadrants.
 * @param regions_to_evaluate Queue reference where children should be added.
 */
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

/**
 * @brief For a given region, get vertex positions for each edge of the region.
 * 
 * @param region The region pointer.
 * @return std::unordered_map<regionEdge,std::vector<StateVec>> the Vertex positions
 */
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

void Quadtree::setCustomVertex(Vertex* s){
    Region* leaf_region = getLeafRegionContaining(s->state.x(),s->state.y());

    gm_->addVertex(s);
    for(auto leaf_vertex_it=leaf_region->vertices.begin(); leaf_vertex_it!=leaf_region->vertices.end(); leaf_vertex_it++){
        double distance;
        geod_.Inverse(s->state.y(),s->state.x(),(*leaf_vertex_it)->state.y(),(*leaf_vertex_it)->state.x(),distance);
        distance = abs(distance);
        gm_->addEdge(s,*leaf_vertex_it,distance);
    }
}

void Quadtree::setStart(double lon, double lat){
    Vertex* s = new Vertex(gm_->generateVertexID(),StateVec(lon,lat,0,0));
    setCustomVertex(s);
    return;
}

void Quadtree::setGoal(double lon,double lat){
    Vertex* g = new Vertex(gm_->generateVertexID(),StateVec(lon,lat,0,0));
    setCustomVertex(g);
    return;
}

/**
 * @brief Build the Quadtree
 * 
 */
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
        } else if(occupied_ratio==0 ){
            //Area is free, add vertecies
            std::unordered_map<regionEdge,std::vector<StateVec>> frame_points = getFramePoints(current_region);
            Vertex* corner;
            for (auto side_it = frame_points.begin(); side_it!=frame_points.end(); side_it++){
                for (auto state_it = (*side_it).second.begin(); state_it != (*side_it).second.end(); state_it++){
                    if (!gm_->getNearestVertexInRange(&(*state_it),0.000001,&corner)){
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

                    double distance;
                    double lon_a = (*it_vert_a)->state.x();
                    double lat_a = (*it_vert_a)->state.y();
                    double lon_b = (*it_vert_b)->state.x();
                    double lat_b = (*it_vert_b)->state.y();
                    geod_.Inverse(lat_a,lon_a,lat_b,lon_b,distance);
                    distance=abs(distance);
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





