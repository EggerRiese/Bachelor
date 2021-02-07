#include "CutoutBoundingBox.h"

int main(){
    int nummer = 8;
    for (int i = 1; i < 4; i++)
    {
        cout << "Aufnahme " + std::to_string(nummer) + "_" + std::to_string(i) << endl;
        cout << "#################################" << endl;
        string filepath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/raw_pointcloud_data/bag/new/" + std::to_string(nummer) + "_"+ std::to_string(i) + ".bag";
        string xmlPath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/raw_pointcloud_data/png/" + std::to_string(nummer) + "_" + std::to_string(i) + ".xml";
        std::string pcdpath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/raw_pointcloud_data/cbrg_output/" + std::to_string(nummer) + "_" + std::to_string(i);
        
        pcl::PointCloud<pclPoint>::Ptr cloud(new pcl::PointCloud<pclPoint>);
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labledCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
        pcl::PointCloud<pclPoint>::Ptr _tempCloud(new pcl::PointCloud<pclPoint>);
        std::vector<pcl::PointCloud<pclPoint>::Ptr>cloudAlist;
        std::vector<vector<int>> BoundingBoxList;
        std::vector<std::string> ObjectList;
        std::vector<int> _tempVector;

        // set config
        rs2::config cfg;
        cfg.enable_device_from_file(filepath, false);    
        std::vector<rs2::frameset> frames = getFrameList(cfg);

        rs2::spatial_filter spat_filter;
        spat_filter.set_option(RS2_OPTION_HOLES_FILL, 5);

        for (auto frame : frames) {
            //frame = spat_filter.process(frame);
            //Generate colored PointCloud from single frame
            auto pcloud = calculateColorPointcloud(frame);
            //pclfuncs::showCloud(pcloud, "BB");
            cloudAlist.push_back(pcloud);
        }    

        //*cloud = *pclfuncs::getMedianXYZCloud(&cloudAlist);
        cloud = cloudAlist[4];
    
        //Read XML
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(xmlPath.c_str());
        if (!result)
            return -1;

        for (pugi::xml_node object: doc.child("annotation").children("object"))
        {    
            ObjectList.push_back(object.child("name").first_child().value());
            auto bnd = object.child("bndbox");
            _tempVector.push_back(std::stoi(bnd.child("xmin").first_child().value()));
            _tempVector.push_back(std::stoi(bnd.child("ymin").first_child().value()));
            _tempVector.push_back(std::stoi(bnd.child("xmax").first_child().value()));
            _tempVector.push_back(std::stoi(bnd.child("ymax").first_child().value()));
            BoundingBoxList.push_back(_tempVector);
            _tempVector.clear();
        }
        
        //remove the background
        //cloud equals the plate
        pcl::PointCloud<pclPoint>::Ptr cloudForMedianGround(new pcl::PointCloud<pclPoint>);
        pcl::copyPointCloud(*cloud, *cloudForMedianGround);
        
        //Copy PointCloud x times
        std::vector<pcl::PointCloud<pclPoint>::Ptr> PointCloudList;
        for (std::vector<int> BoundingBox : BoundingBoxList){
            pcl::PointCloud<pclPoint>::Ptr _tempCloud(new pcl::PointCloud<pclPoint>);
            pcl::copyPointCloud(*cloud, *_tempCloud);
            PointCloudList.push_back(_tempCloud);
        }

        // calculate median
        double median = calculateMedian(pclfuncs::colorBasedRegionGrowing(cloudForMedianGround, 11, 3, 5, false, true, true)); // 11, 3, 5
        int counter = 0;

        // cutout wanted object
        for (pcl::PointCloud<pclPoint>::Ptr cloud1 : PointCloudList) {            
            
            //cutout the wanted object
            //use bounding box data to cut off the unnecessary part of pointcloud
            //returns the pointcloud as bounding box
            pcl::PointCloud<pclPoint>::Ptr bbCloud = cutoutBoundingBox(BoundingBoxList[counter][0], BoundingBoxList[counter][1], BoundingBoxList[counter][2], BoundingBoxList[counter][3], cloud1);
            //pclfuncs::showCloud(bbCloud, "BB");
            bbCloud = pclfuncs::colorBasedRegionGrowing(bbCloud,11,3,5, true);
            //remove unnecessary part of bounding box by color
            //loops through different settings to find biggest cluster
            if (bbCloud->points.size() != 0) {
                bbCloud = pclfuncs::colorBasedRegionGrowing(bbCloud,10, 10,10, false, true, true); 
                bbCloud = pclfuncs::colorBasedRegionGrowing(bbCloud,10,6,7, false, true, true);
                bbCloud = pclfuncs::colorBasedRegionGrowing(bbCloud,10,3,7, false, true, true);
            }    
            
            pclfuncs::showCloud(bbCloud, "BB");
            
            //VolumeCalculation
            /* double volume = 0;
            pclfuncs::qhullDelaunayTriangulation(bbCloud, volume, median);
            cout << "===================" << endl;
            cout << ObjectList[counter] << ": VOLUME: " << volume << endl;
            cout << "===================" << endl; */

            //int correctPoints = findCorrectLabel(ObjectList[counter], bbCloud);

            pcl::io::savePCDFileASCII (pcdpath + "_" + ObjectList[counter] + ".pcd", *bbCloud);

            //Punkte zählen
            cout << "===================" << endl;
            cout << ObjectList[counter] << ": POINTS: " << bbCloud->points.size() << endl;
            //cout << "correct: " << correctPoints << endl;
            cout << "===================" << endl;
            counter ++;
        }
               
    } 

    return 0;
}


int findCorrectLabel(std::string object, pcl::PointCloud<pclPoint>::Ptr cloud) {
    int labelNumber = labelToInt(object);

    for (int i = 0; i < cloud->points.size(); i++) {
        cout << cloud->points.at(i) << endl;
    }
    return 0;
}


int labelToInt(std::string object) {
    if (object.compare("apple")) { return 1;} 
    else if (object == "bread") { return 2;}
    else if (object == "banana") { return 2;}
    else if (object == "cereals") { return 2;}
    else if (object == "cheese") { return 2;}
    else if (object == "cucumber") { return 2;}
    else if (object == "cup_drink") { return 2;}
    else if (object == "egg") { return 2;}
    else if (object == "glass_drink") { return 2;}
    else if (object == "tomato") { return 2;}
    else if (object == "yoghurt") { return 2;}
    else if (object == "sausage") { return 2;}
    return 0;
    
}

pcl::PointCloud<pclPoint>::Ptr cutoutBoundingBox(float bbStartX, float bbStartY, float bbEndX, float bbEndY, pcl::PointCloud<pclPoint>::Ptr cloud){	
    pcl::PointCloud<pclPoint>::Ptr cloud_xFiltered(new pcl::PointCloud<pclPoint>); //x coordinates have been filtered and saved in temp point cloud
	pcl::PointCloud<pclPoint>::Ptr cloud_bb(new pcl::PointCloud<pclPoint>); //point cloud that represents the given BoundingBox

    try {
        //transform coordinates, because origin is centered
        bbStartX = bbStartX - (rsImageWidth/2);
        bbEndX = bbEndX - (rsImageWidth/2);
        bbStartY = (rsImageHeight/2) - bbStartY;
        bbEndY = (rsImageHeight/2) - bbEndY;

        //offset of PointCloud

        /* double xStartOffset = bbStartX * xFactor1;
        double xEndOffset = bbEndX * xFactor2;
        double yStartOffset = bbStartY * yFactor1;
        double yEndOffset = bbEndY * yFactor2; */

        /* bbStartX += xStartOffset;
        bbEndX += xEndOffset; */
        /* if (bbStartY > 0) {
            bbStartY -= yStartOffset;
        } else if (bbStartY < 0) {
            bbStartY += yStartOffset;
        }
        if (bbEndY > 0) {
            bbEndY -= yEndOffset;
        } else  if (bbEndY < 0) {
            bbEndY += yEndOffset;
        }

        if (bbStartX > 0) {
            bbStartX = bbStartX / 2900;
        } else if (bbStartX < 0) {
            bbStartX = bbStartX / 1000;
        }
         if (bbEndX > 0) {
            bbEndX += yEndOffset;
        } else  if (bbEndX < 0) {
            bbEndX -= yEndOffset;
        }  */
        

        //normailze coordinates
        //bbStartX = bbStartX / 1700;
        bbEndX = bbEndX / 1600;
        bbStartY = bbStartY / 1500;
        bbEndY = bbEndY / 1500;

        //invert x axis
        bbStartY = bbStartY * -1;
        bbEndY = bbEndY * -1;


        // Create the filtering object for x values
        pcl::PassThrough<pclPoint> passX;
        passX.setInputCloud (cloud);
        passX.setFilterFieldName ("x");
        passX.setFilterLimits (bbStartX, bbEndX);                
        passX.filter(*cloud); 
        //pclfuncs::showCloud(cloud, "x filtered");

        // Change the filtering object to y values
        pcl::PassThrough<pclPoint> passY;
        passY.setInputCloud(cloud);
        passY.setFilterFieldName("y");
        passY.setFilterLimits(bbStartY, bbEndY);
        passY.filter(*cloud); 
        //pclfuncs::showCloud(cloud, "y");  
        
        return cloud;
    }
    catch (exception e) {
        cout << "Error in calculation: " << e.what() << endl;
        return 0;
    }    
}


/* Returns one Frame with depth and color info */
rs2::frameset getFrames(rs2::config cfg) {

    rs2::pipeline pipe;
    pipe.start(cfg); //start recording file
    rs2::pointcloud pc;
    
    auto profile = pipe.get_active_profile();
	auto device = profile.get_device();
    auto sensor = device.first<rs2::depth_sensor>();
    auto col  = device.first<rs2::color_sensor>();
    depth_units = sensor.get_option(RS2_OPTION_DEPTH_UNITS);
	auto playback = device.as<rs2::playback>();
	playback.pause();
	playback.seek(std::chrono::nanoseconds(0));
	playback.set_real_time(false);
	playback.resume();

    
    // Wait for frames from the camera to settle
    for (int i = 0; i < 5; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }

    // Capture a single frame and obtain depth + RGB values from it    
    auto frames = pipe.wait_for_frames();
    
    pipe.stop(); //stop recording files
    
    return frames;    
}

//Get the list of frames for a rs2::config (only works for recordings)
std::vector<rs2::frameset> getFrameList(rs2::config cfg) {
    std::vector<rs2::frame> frameList;
    std::set<int> numbers;
    std::vector<rs2::frameset> framesetList;

    rs2::pipeline pipe;
    pipe.start(cfg); //start recording file
    rs2::frameset frames;

    auto profile = pipe.get_active_profile();
	auto device = profile.get_device();
    auto sensor = device.first<rs2::depth_sensor>();
    depth_units = sensor.get_option(RS2_OPTION_DEPTH_UNITS);
	auto playback = device.as<rs2::playback>();
	playback.pause();
	playback.seek(std::chrono::nanoseconds(0));
	playback.set_real_time(false);
	playback.resume();

    while(pipe.try_wait_for_frames(&frames,1000) && frameList.size() < 5)
    {
        frames.keep();
        auto fr = pipe.wait_for_frames();
        rs2::depth_frame dFrame = frames.get_depth_frame();
        auto ret = numbers.insert(dFrame.get_frame_number());
        framesetList.push_back(fr);
        if(ret.second) {
            frameList.push_back(dFrame);
        }
    }

    pipe.stop(); //stop recording files
    return framesetList;
}




//Saves IntelRealsense color Image as .png to given path 
void savePng(rs2::frameset frames){
    std::stringstream png_file;
    auto colorImage = frames.get_color_frame();

    png_file << "MorphSnakes/images/rs_" << path << ".png";
    stbi_write_png(png_file.str().c_str(), colorImage.get_width(), colorImage.get_height(),
               colorImage.get_bytes_per_pixel(), colorImage.get_data(), colorImage.get_stride_in_bytes());
    std::cout << "Saved " << png_file.str() << std::endl;
}

double calculateMedian(pcl::PointCloud<pclPoint>::Ptr cloud_plane){
    std::vector<double> med;
    for (int i = 0; i < cloud_plane->points.size(); i++) {
        med.push_back(cloud_plane->points.at(i).z);
    }

    std::sort(med.begin(), med.end());
    double median;
    int index = med.size() / 2;
    if (med.size() % 2 == 1) {
        median = med.at(index);
    }
    else {
        median = (med.at(index - 1) + med.at(index)) / 2;
    }

    return median;
}


double planeSegmentation(pcl::PointCloud<pclPoint>::Ptr cloud, bool pp){
    //object after segmentation
	pcl::PointCloud<pclPoint>::Ptr cloud_f(new pcl::PointCloud<pclPoint>);
	//plane after segmentation
    pcl::PointCloud<pclPoint>::Ptr cloud_plane(new pcl::PointCloud<pclPoint>);     
    
    try {        
        //might have zeros in point cloud
        if (!pp) {
            pclfuncs::removeZeroFromPointCloud(*cloud, *cloud);
        }

        //SOR Before
        pclfuncs::statisticalOutlierRemovel(cloud);

        //Execute plane model segmentation
        if (!pclfuncs::planeModelSegmentation(cloud, *cloud_f, *cloud_plane)) {
            cout << "Cannot segment plane!" << endl << endl;
        }
        pclfuncs::showCloud(cloud,"cloud");
        pclfuncs::showCloud(cloud_f,"cloud after plane segmentation");
        pclfuncs::showCloud(cloud_plane,"cloud_plane");

        //calculation median ground
        std::vector<double> med;
        for (int i = 0; i < cloud_plane->points.size(); i++) {
            med.push_back(cloud_plane->points.at(i).z);
        }
        std::sort(med.begin(), med.end());
        double median;
        if(cloud_plane->points.size() < cloud->points.size()/2){
            //if plane segmentation was successful
            //do as programmed by Simon
            int index = med.size() / 2;
            if (med.size() % 2 == 1) {
                median = med.at(index);
            }
            else {
                median = (med.at(index - 1) + med.at(index)) / 2;
            }
        }else {
            //if plane segmentation was not successful
            //use only the smalest 
            int index = med.size() / 1.5;
            if (med.size() % 2 == 1) {
                median = med.at(index);
            }
            else {
                median = (med.at(index - 1) + med.at(index)) / 2;
            }
        }
        //pclfuncs::showCloud(cloud_f, "plane segmentation");
        cout << "MEDIAN GROUND: " << median << endl;
        return median;
    }
    catch (exception e) {
        cout << "Error in calculation: " << e.what() << endl;
        return 0;
    }
		
}

//Apply filter to frame and return it
rs2::frame processFilters(rs2::frame frame) {
    frame = depth_to_disparity.process(frame);
    //frame = spat_filter.process(frame);
    frame = disparity_to_depth.process(frame);
    return frame;
}

void stepThroughPointCloud(pcl::PointCloud<pclPoint>::Ptr cloud, double median){
    pcl::PointCloud<pclPoint>::Ptr temp_cloud(new pcl::PointCloud<pclPoint>);
    auto zList = getLayer(cloud);

    for(int i = zList.size()-1; i > 0; i--){
        pcl::PassThrough<pclPoint> passX;
        passX.setInputCloud (cloud);
        passX.setFilterFieldName ("z");
        passX.setFilterLimits (0, median);                
        passX.filter(*cloud);
        
    }
    pclfuncs::showCloud(cloud, "median entfern");
}

std::vector<float> getLayer(pcl::PointCloud<pclPoint>::Ptr cloud){
    std::vector<float> zIndices;
    for (int i = 0; i < cloud->points.size(); i++){
        if(std::find(zIndices.begin(), zIndices.end(), cloud->points[i].z) != zIndices.end()) {
            /* zIndices contains z */
        } else {
            zIndices.push_back(cloud->points[i].z);
        }
    }
    std::sort(zIndices.begin(), zIndices.end());
    return zIndices;
}