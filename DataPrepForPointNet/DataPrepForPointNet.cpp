#include "DataPrepForPointNet.h"


int main (int argc, char** argv)
{
    //generatePCD();
    generateSegFiles();
    
    return (0);
}

void generatePCD(){
    for (int i = 1; i <= 9; i++)
    {
        std::string mainPath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/train/yoghurt/";

        rs2::config cfg;
        std::string bagPath = mainPath + std::to_string(i) + ".bag";

        cfg.enable_device_from_file(bagPath, false);
        pcl::PointCloud<pclPoint>::Ptr cloud(new pcl::PointCloud<pclPoint>);
        rs2::frameset frame = getFrames(cfg, 3);

        std::string pathForFrame = mainPath + std::to_string(i);

        //savePng(frame, pathForFrame);

        //Generate colored PointCloud from single frame
        *cloud = *calculateColorPointcloud(frame);
        //pclfuncs::showCloud(cloud, "first");

        pcl::PointCloud<pclPoint>::Ptr downsampled_cloud_PointCloud(new pcl::PointCloud<pclPoint>);
        pcl::PointCloud<pclPoint>::Ptr cloud_plane(new pcl::PointCloud<pclPoint>);
        pcl::PointCloud<pclPoint>::Ptr cloud_f(new pcl::PointCloud<pclPoint>);

        pclfuncs::removeZeroFromPointCloud(*cloud, *cloud);
    
        //SOR Before
        pclfuncs::statisticalOutlierRemovel(cloud);            
        
        //*cloud = *removeGroundPlane(cloud);
        //pclfuncs::planeModelSegmentation(cloud, *cloud_f, *cloud_plane);
        pclfuncs::colorBasedRegionGrowing(cloud, 11, 3, 5, true, false, false);
        pclfuncs::showCloud(cloud, std::to_string(i));

        int s = cloud->points.size();
        cout << s << endl;

        // Create the filtering object
        pcl::VoxelGrid<pclPoint> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.001f, 0.001f, 0.001f);
        sor.filter(*downsampled_cloud_PointCloud);   
        s = downsampled_cloud_PointCloud->points.size();
        //pclfuncs::showCloud(downsampled_cloud_PointCloud, "befor downsampling");
        cout << s << endl;

        while (downsampled_cloud_PointCloud->points.size() > 4096) {
            downsampled_cloud_PointCloud->points.pop_back();
        }
        downsampled_cloud_PointCloud->width = 4096;
        downsampled_cloud_PointCloud->height = 1;
        downsampled_cloud_PointCloud->points.resize(4096 * 1);
        //pclfuncs::showCloud(downsampled_cloud_PointCloud, "downsampled");

        std::string pcd_path  = mainPath + std::to_string(i) + ".pcd";
        //pcd_path += "_";
        //pcd_path += std::to_string(k);
        //pcd_path += ".pcd";
        savePCDFileASCII(pcd_path, *downsampled_cloud_PointCloud);    
    }
    
}

void generateSegFiles(){
    pcl::PCLPointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZL> );
    pcl::PCDReader reader;

    std::string mainPath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/train/apple/second_try/";

    for (directory_entry& entry : directory_iterator(mainPath)){
        const boost::filesystem::path path = entry.path();
        std::string path_string =  path.string();
        std::string org_path = path_string;
        std::cout << path_string << '\n';
        size_t i = 0; 

        for ( ; i < path_string.length(); i++ ){ if ( isdigit(path_string[i]) ) break; }

        // remove the first chars, which aren't digits
        path_string = path_string.substr(i, path_string.length() - i );

        // convert the remaining text to an integer
        int id = atoi(path_string.c_str());
        std::cout  << id << std::endl;

        if (reader.read(org_path, cloud, 0) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file %s \n",org_path.c_str());
            continue;
        }
        pcl::fromPCLPointCloud2(cloud, *vertices );
        
        
        std::ofstream MyFile(mainPath + std::to_string(id) + ".txt");
        for(auto& p : vertices->points){
            MyFile << p.label << "\n";
        }
        MyFile.close();
    }
}

pcl::PointCloud<pclPoint>::Ptr removeGroundPlane(pcl::PointCloud<pclPoint>::Ptr cloud) {
    pcl::PointCloud<pclPoint>::Ptr cloud_filtered(new pcl::PointCloud<pclPoint>);
    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    cloud_filtered->header = cloud->header;
    cloud_filtered->height = 1;
    cloud_filtered->width = cloud_filtered->points.size();  
    return cloud_filtered;
}

//Saves IntelRealsense color Image as .png to given path 
void savePng(rs2::frameset frames, std::string path){
    std::stringstream png_file;
    auto colorImage = frames.get_color_frame();

    png_file << path << ".png";
    stbi_write_png(png_file.str().c_str(), colorImage.get_width(), colorImage.get_height(),
               colorImage.get_bytes_per_pixel(), colorImage.get_data(), colorImage.get_stride_in_bytes());
    std::cout << "Saved " << png_file.str() << std::endl;
}

/* Returns one Frame with depth and color info */
rs2::frameset getFrames(rs2::config cfg, int frameNumber) {

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
    for (int i = 0; i < frameNumber; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }

    // Capture a single frame and obtain depth + RGB values from it    
    auto frames = pipe.wait_for_frames();
    
    pipe.stop(); //stop recording files
    
    return frames;    
}