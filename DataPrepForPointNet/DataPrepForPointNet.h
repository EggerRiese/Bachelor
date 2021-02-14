#pragma once

#include <fstream> 
#include <iostream>
#include <pclfuncs.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/filesystem.hpp>
#include <pwd.h>
#include <variables.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "/home/joshua/Dokumente/stb/stb_image_write.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace boost::filesystem;

double depth_units;


void generatePCD();
void generateSegFiles();
void savePng(rs2::frameset frames, std::string path);
pcl::PointCloud<pclPoint>::Ptr removeGroundPlane(pcl::PointCloud<pclPoint>::Ptr cloud);
void downsamplePCL(pcl::PointCloud<pclPoint> cloud, pcl::PointCloud<pclPoint>::Ptr cloud_filtered);
rs2::frameset getFrames(rs2::config cfg, int frameNumber);