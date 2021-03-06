#include "usv_map/quadtree.h"

/**
 * @brief Construct a new Quadtree object
 * 
 * @param lower_left The lower left point defining the region in which the quadtree is to be built.
 * @param upper_right The upper right point defining the region in which the quadtree is to be built.
 * @param ds The dataset containing the map we build the Quadtree w.r.t. 
 * @param build_immediately If true, build the quadtree. If false, dont.
 */
Quadtree::Quadtree(OGRPoint lower_left, OGRPoint upper_right, GDALDataset* ds, GDALDataset* ds_detailed, std::string mission_region,MapService* map_service,bool build_immediately): 
    ds_(ds),
    ds_detailed_(ds_detailed),
    lower_left_(lower_left),
    upper_right_(upper_right),
    geod_(GeographicLib::Geodesic::WGS84()),
    mission_region_(mission_region),
    map_service_(map_service){
    gm_ = new GraphManager;

    //Load parameters
    //Load parameters
    bool parameter_load_error = false;
    if(!ros::param::get("map_preprocessing/quadtree/fixed_divisor_flag",fixed_divisor_flag_)) parameter_load_error = true;
    if(!ros::param::get("map_preprocessing/quadtree/fixed_divisor_value",fixed_divisor_value_)) parameter_load_error = true;
    if(!ros::param::get("map_preprocessing/quadtree/max_length_divisor_value",max_length_divisor_value_)) parameter_load_error = true;
    if(parameter_load_error){
        ROS_ERROR_STREAM("Failed to load a parameter");
        ros::shutdown();
    }
    benchmark_data_.vertices=0;
    ros::Time start = ros::Time::now();
    if (build_immediately){
        build();
        save(mission_region);
        benchmark_data_.build_time = ros::Duration(ros::Time::now()-start).toSec();
        dumpBenchmark();
    } 
    else load(mission_region);
    ros::Time end = ros::Time::now();

    
}

/**
 * @brief Save the quadtree for later loading or post-mission visualization.
 * 
 * @param tree_name What the quadtree should be saved as (will overwrite any previous stored tree witht his name)
 */
void Quadtree::save(const std::string& mission_region){
    std::string path = ros::package::getPath("usv_map");
    path.append("/data/mission_regions/"+mission_region+"/");
    //Save graph

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    std::string graph_path = path + "quadtree_graph";
    gm_->saveGraph(graph_path);

    //Save for post-mission visualization
    std::string viz_path = path+"quadtree.csv";
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
            layer->CreateField(new OGRFieldDefn("is_leaf",OGRFieldType::OFTInteger64));
        }
        OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetGeometry(current->region_polygon_);
        feature->SetField("parent",current->getParentID());
        feature->SetField("id",current->getID());
        feature->SetField("region",static_cast<int>(current->getOwnRegion()));
        feature->SetField("is_leaf",static_cast<int>(current->is_leaf_));
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

void Quadtree::saveForVisualization(const std::string& quadtree_name){
    std::string path = ros::package::getPath("usv_map");
    path.append("/data/mission_regions/"+mission_region_+"/");
    //Save graph

    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directory(path);
    }

    //Save for post-mission visualization
    std::string viz_path = path+quadtree_name+".csv";
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
}

/**
 * @brief Load a saved quadtree.
 * 
 * @param tree_name Nam eof the saved quadtree. If a non-existent quadtree is named, the code will crash hard!
 */
void Quadtree::load(const std::string& mission_region){
    std::string path = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region+"/";
    std::string graph_path = path + "quadtree_graph";
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

                tree_root_ = new Region(lower_left_,upper_right_,current_depth,generateRegionID(),0,childRegion::NW,ds_,ds_detailed_,map_service_);
                
                //Add region to parent_lookup 
                parent_lookup[tree_root_->getID()]=tree_root_;
            } else{
                Region* parent = parent_lookup[feat->GetFieldAsInteger64("parent")];
                switch (static_cast<childRegion>(feat->GetFieldAsInteger64("region")))
                {
                case childRegion::NW:
                    parent->addChild(new Region(parent->lower_left_.getX(),parent->lower_left_.getY()+parent->getHeight()/2,parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,feat->GetFieldAsInteger64("id"),parent->getID(),childRegion::NW,ds_,ds_detailed_,map_service_),childRegion::NW);
                    break;
                case childRegion::NE:
                    parent->addChild(new Region(parent->lower_left_.getX()+parent->getWidth()/2,parent->lower_left_.getY()+parent->getHeight()/2,parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,feat->GetFieldAsInteger64("id"),parent->getID(),childRegion::NE,ds_,ds_detailed_,map_service_),childRegion::NE);
                    break;
                case childRegion::SW:
                    parent->addChild(new Region(parent->lower_left_.getX(),parent->lower_left_.getY(),parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,feat->GetFieldAsInteger64("id"),parent->getID(),childRegion::SW,ds_,ds_detailed_,map_service_),childRegion::SW);
                    break;
                case childRegion::SE:
                    parent->addChild(new Region(parent->lower_left_.getX()+parent->getWidth()/2,parent->lower_left_.getY(),parent->getWidth()/2,parent->getHeight()/2,parent->getDepth()+1,feat->GetFieldAsInteger64("id"),parent->getID(),childRegion::SE,ds_,ds_detailed_,map_service_),childRegion::SE);
                    break;
                default:
                    ROS_ERROR_STREAM("Region type not recognized!");
                    break;
                }
                //Add child to parent map
                parent_lookup[parent->getChildRegion(static_cast<childRegion>(feat->GetFieldAsInteger64("region")))->getID()]=parent->getChildRegion(static_cast<childRegion>(feat->GetFieldAsInteger64("region")));
                Region* child = parent->getChildRegion(static_cast<childRegion>(feat->GetFieldAsInteger64("region")));
                child->is_leaf_=static_cast<bool>(feat->GetFieldAsInteger64("is_leaf"));
                if(child->is_leaf_){
                    std::unordered_map<regionEdge,std::vector<StateVec>> frame_states = getFramePoints(parent->getChildRegion(static_cast<childRegion>(feat->GetFieldAsInteger64("region"))));
                    Vertex* tmp;
                    for(auto frame_state_it=frame_states.begin(); frame_state_it!=frame_states.end(); frame_state_it++){
                        for(auto state_it=(*frame_state_it).second.begin(); state_it!=(*frame_state_it).second.end(); state_it++){
                            gm_->getNearestVertex(&(*state_it),&tmp);
                            child->vertices.push_back(tmp);
                        }
                    }
                }
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

void Quadtree::dumpBenchmark(){
    std::string path = ros::package::getPath("usv_map")+"/data/mission_regions/"+mission_region_+"/benchmark/quadtree/";
    if(!boost::filesystem::exists(path)){
        boost::filesystem::create_directories(path);
    }

    std::ofstream benchmark_file_time(path+"benchmark_time.csv");
    std::ofstream benchmark_file_misc(path+"benchmark_misc.csv");
    benchmark_file_time<<"name,times_run,total_time\n";
    benchmark_file_misc<<"name,value\n";

    std::cout << "Buildtime total: " << benchmark_data_.build_time << std::endl;
    std::cout << "Regions: " << benchmark_data_.splitRegion_time.size()*4 << std::endl;
    std::cout << "Vertices: " << benchmark_data_.vertices << std::endl;
    std::cout << "Region build time total: " << std::accumulate(benchmark_data_.splitRegion_time.begin(),benchmark_data_.splitRegion_time.end(),0.0) << std::endl;
    std::cout << "Get occupancy of region total: " << std::accumulate(benchmark_data_.getOccupiedArea_time.begin(),benchmark_data_.getOccupiedArea_time.end(),0.0) << std::endl;
    std::cout << "Get Frame points: " << std::accumulate(benchmark_data_.getFramePoints_time.begin(),benchmark_data_.getFramePoints_time.end(),0.0) << std::endl;

    benchmark_file_misc<<"regions"<<","<< benchmark_data_.splitRegion_time.size()*4 << "\n";
    benchmark_file_misc<<"vertices"<<","<< benchmark_data_.vertices << "\n";
    benchmark_file_time<<"build"<<","<<1<<","<<benchmark_data_.build_time<<"\n";
    benchmark_file_time<<"get_occipied_area"<<","<<benchmark_data_.getOccupiedArea_time.size()<<","<<std::accumulate(benchmark_data_.getOccupiedArea_time.begin(),benchmark_data_.getOccupiedArea_time.end(),0.0)<<"\n";
    benchmark_file_time<<"split_region"<<","<<benchmark_data_.splitRegion_time.size()<<","<<std::accumulate(benchmark_data_.splitRegion_time.begin(),benchmark_data_.splitRegion_time.end(),0.0)<<"\n";
    benchmark_file_time<<"get_frame_points"<<","<<benchmark_data_.getFramePoints_time.size()<<","<<std::accumulate(benchmark_data_.getFramePoints_time.begin(),benchmark_data_.getFramePoints_time.end(),0.0)<<"\n";


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
    while(current!=prev && current!=nullptr){
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
    region->addChild(new Region(region->lower_left_.getX(),region->lower_left_.getY()+region->getHeight()/2,region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::NW,ds_,ds_detailed_,map_service_),childRegion::NW);
    //Calculate NE region
    region->addChild(new Region(region->lower_left_.getX()+region->getWidth()/2,region->lower_left_.getY()+region->getHeight()/2,region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::NE,ds_,ds_detailed_,map_service_),childRegion::NE);
    //Calculate SW region
    region->addChild(new Region(region->lower_left_.getX(),region->lower_left_.getY(),region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::SW,ds_,ds_detailed_,map_service_),childRegion::SW);
    //Calculate SE region
    region->addChild(new Region(region->lower_left_.getX()+region->getWidth()/2,region->lower_left_.getY(),region->getWidth()/2,region->getHeight()/2,region->getDepth()+1,generateRegionID(),region->getID(),childRegion::SE,ds_,ds_detailed_,map_service_),childRegion::SE);
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
    ros::Time start = ros::Time::now();
    std::unordered_map<regionEdge,std::vector<StateVec>> frame_points;
    int divisor_NS, divisor_EW;
    if(!fixed_divisor_flag_){
        //Determine divisor for N/S edge
        double length_NS;
        geod_.Inverse(region->lower_left_.getY(),region->lower_left_.getX(),region->lower_left_.getY(),region->upper_right_.getX(),length_NS);
        length_NS = abs(length_NS);
        divisor_NS = std::round(length_NS/max_length_divisor_value_+0.5);
        if(divisor_NS!=1){
            int i = 1;
            while(divisor_NS!=pow(2,i)){
                if(divisor_NS<pow(2,i)){
                    divisor_NS = pow(2,i);
                } else{
                    i++;
                }
            }
        }

        //Determine divisor for E/W edge
        double length_EW;
        geod_.Inverse(region->lower_left_.getY(),region->lower_left_.getX(),region->upper_right_.getY(),region->lower_left_.getX(),length_EW);
        length_EW = abs(length_EW);
        divisor_EW = std::round(length_EW/max_length_divisor_value_+0.5);
        if(divisor_EW!=1){
            int i = 1;
            while(divisor_EW!=pow(2,i)){
                if(divisor_EW<pow(2,i)){
                    divisor_EW = pow(2,i);
                } else{
                    i++;
                }
            }
        }
    } else{
        divisor_NS = fixed_divisor_value_;
        divisor_EW = fixed_divisor_value_;
    }

    //Determine points for south edge and north edge
    for (double x=region->lower_left_.getX(); x<=region->upper_right_.getX();x+=region->getWidth()/divisor_NS){
        //South
        frame_points[regionEdge::S].push_back(StateVec(x,region->lower_left_.getY(),0,0));
        //North
        frame_points[regionEdge::N].push_back(StateVec(x,region->upper_right_.getY(),0,0));
        benchmark_data_.vertices+=2;
    }

    //Determine points for west and east edge
    for (double y=region->lower_left_.getY(); y<=region->upper_right_.getY();y+=region->getHeight()/divisor_EW){
        //West
        frame_points[regionEdge::W].push_back(StateVec(region->lower_left_.getX(),y,0,0));
        //East
        frame_points[regionEdge::E].push_back(StateVec(region->upper_right_.getX(),y,0,0));
        benchmark_data_.vertices+=2;
    }
    benchmark_data_.getFramePoints_time.push_back(ros::Duration(ros::Time::now()-start).toSec());
    return frame_points;
}

/**
 * @brief Introduce a custom vertex to the graph and make connection with all edges in same region.
 * 
 * @param s Vertex
 */
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

/**
 * @brief Set start
 * 
 * @param lon Longitude
 * @param lat Latitude
 */
void Quadtree::setStart(double lon, double lat){
    Vertex* s = new Vertex(gm_->generateVertexID(),StateVec(lon,lat,0,0));
    setCustomVertex(s);
    return;
}

/**
 * @brief Set goal
 * 
 * @param lon Longitude
 * @param lat Latitude
 */
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
    tree_root_ = new Region(lower_left_,upper_right_,0,generateRegionID(),0,childRegion::NW,ds_,ds_detailed_,map_service_);
    regions_to_evaluate.push(tree_root_);
    int regions_added = 0;
    while(!regions_to_evaluate.empty()){
        Region* current_region = regions_to_evaluate.front();
        regions_to_evaluate.pop();
        if(current_region->getArea()<pow(10,-7)){
            continue;
        }

        ros::Time start = ros::Time::now();
        double occupied_ratio = current_region->getOccupiedRatio();
        //bool obstacle_free = current_region->obstacleFree();
        ros::Time end = ros::Time::now();
        benchmark_data_.getOccupiedArea_time.push_back(ros::Duration(end-start).toSec());
        
        if(occupied_ratio==1){
            continue;
        }else if(occupied_ratio==0){
            regions_added++;
            std::cout << "Regions added" << regions_added << std::endl;
            //Area is free, add vertecies
            current_region->is_leaf_=true;
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





