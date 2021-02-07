#pragma once


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "/home/joshua/Dokumente/Bachelor/stb-master/stb_image_write.h"
#include "pugixml.hpp"
#include <iostream>
#include <string>
#include <librealsense2/rs.hpp> // Intel Realsense Cross Plattform API
#include <thread>
#include <pclfuncs.h>
#include <fstream>
#include <pcl/filters/passthrough.h>
#include <pcl/common/projection_matrix.h>
#include <stdlib.h>
#include <variables.h>
#include <array>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <map>

using namespace std;
using namespace pcl::io;

//const string filepath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/bag/100/2_1.bag";
//const string xmlPath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/png/2_1.xml";
double depth_units;

ofstream dataFile;

rs2::colorizer color_map;
rs2::pipeline rsPipe;

std::vector<rs2::video_frame> cFrameList;

//Post Processing Filters
rs2::spatial_filter spat_filter;
rs2::disparity_transform depth_to_disparity;
rs2::disparity_transform disparity_to_depth;





int labelToInt(std::string object);
void savePng(rs2::frameset frames);
rs2::frameset getFrames(rs2::config cfg);
rs2::frame processFilters(rs2::frame frame);
double planeSegmentation(pcl::PointCloud<pclPoint>::Ptr cloud, bool pp);
pcl::PointCloud<pclPoint>::Ptr cutoutBoundingBox(float bbStartX, float bbStartY, float bbEndX, float bbEndY, pcl::PointCloud<pclPoint>::Ptr cloud);
std::vector<vector<float>> getCoordinates(pcl::PointCloud<pclPoint>::Ptr cloud);
void stepThroughPointCloud(pcl::PointCloud<pclPoint>::Ptr cloud, double median);
std::vector<float> getLayer(pcl::PointCloud<pclPoint>::Ptr cloud);
double calculateMedian(pcl::PointCloud<pclPoint>::Ptr cloud_plane);
std::vector<rs2::frameset> getFrameList(rs2::config cfg);
int findCorrectLabel(std::string object, pcl::PointCloud<pclPoint>::Ptr cloud) ;