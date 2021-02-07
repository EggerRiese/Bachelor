#ifndef PCLFUNCS_H_
#define PCLFUNCS_H_

#include <iostream>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <thread>
#include <cmath> 

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-helpers.h>

#include <libqhullcpp/RboxPoints.h>
#include <libqhullcpp/QhullError.h>
#include <libqhullcpp/QhullQh.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullLinkedList.h>
#include <libqhullcpp/QhullVertex.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullPoint.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetList;
using orgQhull::QhullQh;
using orgQhull::RboxPoints;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexSet;

using namespace std;

//current intel frame
rs2::frame cFrame;

// looping thread running?
bool loop = false; 

//to store a point cloud frome the live cam you can use strg + a | b to save the point cloud to cloudA | cloudB
pcl::PointCloud<pclPoint>::Ptr cloudA(new pcl::PointCloud<pclPoint>);
pcl::PointCloud<pclPoint>::Ptr cloudB(new pcl::PointCloud<pclPoint>);

//for calculation on more than just one point cloud, everytime a point cloud is stored in cloudA | cloudB it will add to the list cloudAlist | cloudBlist
std::vector<pcl::PointCloud<pclPoint>::Ptr> cloudAlist;
std::vector<pcl::PointCloud<pclPoint>::Ptr> cloudBlist;

//mutex for blocking/releasing a visualization thread
std::mutex mut;


namespace pclfuncs {
	//Thread: Save screenshot of current pointcloud with timestamp 
	void saveScreenshot(pcl::PointCloud <pclPoint> s_pc) {
		std::cout << "start saving Pointcloud..." << std::endl;
		time_t now = time(0);
		std::stringstream filename_pcd;
		std::stringstream filename_ply;
		filename_pcd << "savePCL_" << now << ".pcd";
		filename_ply << "savePCL_" << now << ".ply";
		pcl::io::savePCDFileASCII(filename_pcd.str(), s_pc);
		std::cout << "Pointcloud saved at '" << filename_pcd.str() << "'" << std::endl;
		pcl::io::savePLYFileASCII(filename_ply.str(), s_pc);
		std::cout << "Pointcloud saved at '" << filename_ply.str() << "'" << std::endl;
	}

	//Shows a single cloud (stops the main visualizer)
	void showCloud(pcl::PointCloud <pclPoint>::Ptr s_pc, std::string name) {
		pcl::visualization::PCLVisualizer* polyviewer(new pcl::visualization::PCLVisualizer(name));
		polyviewer->setBackgroundColor(255, 255, 255);
		polyviewer->addPointCloud(s_pc);
		//polyviewer->addCoordinateSystem();
		//polyviewer->setCameraPosition(0,0,-1,0,0,0,0,0,0); 
		mut.lock();
		while (!polyviewer->wasStopped()) // Application still alive?
		{
			polyviewer->spinOnce(100);
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		
		mut.unlock();
	}

	//loop for screenshot clouds
	// void cloudLoop() {
	// 	int count = 0;
	// 	while (loop) {
	// 		*cloudA = *(calculatePointcloud(cFrame));
	// 		cloudAlist.push_back(cloudA);
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// 		*cloudB = *(calculatePointcloud(cFrame));
	// 		cloudBlist.push_back(cloudB);
	// 		count++;
	// 		if (count % 10 == 0) {
	// 			std::cout << "[LOOP] " << count << " clouds stored!" << std::endl;
	// 		}
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// 	}
	// }

	//Checking for valid input point cloud list
	bool isPointCloudListValid(std::vector<pcl::PointCloud<pclPoint>::Ptr>* cloudList) {

		pcl::PointCloud<pclPoint> cloud_tmp;

		if (cloudList->size() == 0) {
			return false;
		}
		cloud_tmp = *cloudList->at(0);
		for (int i = 0; i < cloudList->size(); i++) {
			if (cloudList->at(i)->size() != cloud_tmp.size()) {
				return false;
			}
		}
		return true;
	}

	//Take Cloud #0 and set Z value of each Point on AVG(Z)
	pcl::PointCloud<pclPoint>::Ptr getAVGZCloud(std::vector<pcl::PointCloud<pclPoint>::Ptr>* cloudList) {
		pcl::PointCloud<pclPoint>::Ptr result(new pcl::PointCloud<pclPoint>);

		if (isPointCloudListValid(cloudList)) {
			result->width = cloudList->at(0)->width;
			result->height = cloudList->at(0)->height;
			result->resize(result->width*result->height);
		}

		for (int p = 0; p < result->points.size(); p++) {
			double avg = 0.0;
			for (int i = 0; i < cloudList->size(); i++) {
				avg += cloudList->at(i)->points.at(p).z;
			}
			result->points.at(p).x = cloudList->at(0)->points.at(p).x;
			result->points.at(p).y = cloudList->at(0)->points.at(p).y;
			result->points.at(p).z = (avg / cloudList->size());
		}
		return result;
	}

	//Set X,Y,Z value of each Point on Median(X),Median(Y),Median(Z)
	pcl::PointCloud<pclPoint>::Ptr getMedianXYZCloud(std::vector<pcl::PointCloud<pclPoint>::Ptr>* cloudList) {
		pcl::PointCloud<pclPoint>::Ptr result(new pcl::PointCloud<pclPoint>);

		if (isPointCloudListValid(cloudList)) {
			result->width = cloudList->at(0)->width;
			result->height = cloudList->at(0)->height;
			result->resize(result->width*result->height);
		}
		else {
			cout << "CloudList is invalid!" << endl;
			return result;
		}

		for (int p = 0; p < result->points.size(); p++) {
			std::vector<double> med[3];
			double median[3];
			for (int i = 0; i < cloudList->size(); i++) {
				double x = cloudList->at(i)->points.at(p).x;
				double y = cloudList->at(i)->points.at(p).y;
				double z = cloudList->at(i)->points.at(p).z;

				if (x == 0 && y == 0 && z == 0) continue;

				med[0].push_back(x);
				med[1].push_back(y);
				med[2].push_back(z);
			}

			for (int m = 0; m < 3; m++) {

				if (med[m].size() == 0) {
					median[m] = 0;
					continue;
				}
				
				std::sort(med[m].begin(), med[m].end());

				int index = med[m].size() / 2;
				if (med[m].size() % 2 == 1) {
					median[m] = med[m].at(index);
				}
				else {
					median[m] = (med[m].at(index - 1) + med[m].at(index)) / 2;
				}
			}

			result->points.at(p).x = median[0];
			result->points.at(p).y = median[1];
			result->points.at(p).z = median[2];
			result->points.at(p).r = 255;
			result->points.at(p).g = 255;
			result->points.at(p).b = 255;

			//cout << "x: " << median[0] << "    y: " << median[1] << "    z: " << median[2] << endl;
		}
		return result;
	}

	//Set X,Y,Z value of each Point on X(Median(Z)), Y(Median(Z)), Median(Z)
	pcl::PointCloud<pclPoint>::Ptr getMedianZ_XYZCloud(std::vector<pcl::PointCloud<pclPoint>::Ptr>* cloudList) {
		pcl::PointCloud<pclPoint>::Ptr result(new pcl::PointCloud<pclPoint>);

		if (isPointCloudListValid(cloudList)) {
			result->width = cloudList->at(0)->width;
			result->height = cloudList->at(0)->height;
			result->resize(result->width*result->height);
		}

		for (int p = 0; p < result->points.size(); p++) {
			std::list<pclPoint> median;
			for (int i = 0; i < cloudList->size(); i++) {
				median.push_back(cloudList->at(i)->points.at(p));
			}
			median.sort([](const pclPoint & a, const pclPoint & b) { return a.z < b.z; });
			int index = floor(median.size() / 2);
			auto it = median.begin();
			std::advance(it, index);
			result->points.at(p).x = it->x;
			result->points.at(p).y = it->y;
			result->points.at(p).z = it->z;
		}
		return result;
	}

	//Set X,Y,Z value of each Point on AVG(X),AVG(Y),AVG(Z)
	pcl::PointCloud<pclPoint>::Ptr getAVGXYZCloud(std::vector<pcl::PointCloud<pclPoint>::Ptr>* cloudList) {
		pcl::PointCloud<pclPoint>::Ptr result(new pcl::PointCloud<pclPoint>);

		if (isPointCloudListValid(cloudList)) {
			result->width = cloudList->at(0)->width;
			result->height = cloudList->at(0)->height;
			result->resize(result->width*result->height);
		}

		for (int p = 0; p < result->points.size(); p++) {
			double avg[3] = { 0.0, 0.0, 0.0 };
			for (int i = 0; i < cloudList->size(); i++) {
				avg[0] += cloudList->at(i)->points.at(p).x;
				avg[1] += cloudList->at(i)->points.at(p).y;
				avg[2] += cloudList->at(i)->points.at(p).z;
			}
			result->points.at(p).x = (avg[0] / cloudList->size());
			result->points.at(p).y = (avg[1] / cloudList->size());
			result->points.at(p).z = (avg[2] / cloudList->size());
		}
		return result;
	}

	//remove invalid points from point cloud
	template <typename PointT> void
	removeZeroFromPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out) {
		// If the clouds are not the same, prepare the output
		if (&cloud_in != &cloud_out)
		{
			cloud_out.header = cloud_in.header;
			cloud_out.points.resize(cloud_in.points.size());
		}

		size_t j = 0;

		// If the data is dense, we don't need to check for NaN
		for (size_t i = 0; i < cloud_in.points.size(); ++i)
		{
			if (cloud_in.points[i].x == 0 &&
				cloud_in.points[i].y == 0 &&
				cloud_in.points[i].z == 0)
				continue;
			cloud_out.points[j] = cloud_in.points[i];
			j++;
		}
		if (j != cloud_in.points.size())
		{
			// Resize to the correct size
			cloud_out.points.resize(j);
		}

		cloud_out.height = 1;
		cloud_out.width = static_cast<uint32_t>(j);

		// Removing bad points => dense (note: 'dense' doesn't mean 'organized')
		cloud_out.is_dense = true;
	}


	//Calculates volume with delaunay triangulation in relation to a given ground
	void qhullDelaunayTriangulation(pcl::PointCloud<pclPoint>::Ptr cloud, double & volume, double ground = 0.0, double depth_unit = 0.001) {

		//std::cout << "Delaunay Triangulation" << std::endl;

		if (cloud->points.size() < 3) {
			cout << "Pointcloud to small!" << endl;
			return;
		}
		
		
		std::vector<std::pair<double, double>> areaZ;
		orgQhull::Coordinates points;
		for (int i = 0; i < cloud->size(); i++) {
			points.append(cloud->points.at(i).x);
			points.append(cloud->points.at(i).y);
			areaZ.push_back(std::pair<double, double>(0, cloud->points.at(i).z));
		}

		Qhull q;
		RboxPoints rbox;
		rbox.setDimension(2);
		rbox.append(points);
		q.setOutputStream(&std::cout);
		q.runQhull(rbox, "d Qbb Qt");
		//q.outputQhull();
		QhullFacetList facets = q.facetList();

		auto faces = facets.toStdVector();
		for (int i = 0; i < faces.size(); i++) {

			auto face = faces.at(i);
			double triArea = face.facetArea();
			auto vertices = face.vertices().toStdVector();

			if (vertices.size() == 3) {
				double _AB, _AC, _BC, _SUM;

				_AB = vertices.at(0).point().distance(vertices.at(1).point());
				_AC = vertices.at(0).point().distance(vertices.at(2).point());
				_BC = vertices.at(2).point().distance(vertices.at(1).point());
				_SUM = _AB + _AC + _BC;

				//relation and area per point
				double rel[3], areaP[3];
				rel[0] = (_AB + _AC) / _SUM / 2;
				rel[1] = (_AB + _BC) / _SUM / 2;
				rel[2] = (_BC + _AC) / _SUM / 2;
				for (int j = 0; j < 3; j++) {
					areaP[j] = triArea * rel[j];
					areaZ.at(vertices.at(j).point().id()).first +=  areaP[j];
				}
			}
			else {
				std::cout << "Vertices != 3" << std::endl;
			}
		}

		//std::cout << "realArea: " << q.area()*10000 << std::endl;

		double calcArea = 0.0, calcVol = 0.0;
		for (int i = 0; i < areaZ.size(); i++) {
			calcArea += areaZ.at(i).first;
			calcVol += areaZ.at(i).first * abs(areaZ.at(i).second-ground);
		}

		//std::cout << "calcArea: " << calcArea*10000 << std::endl;
		//std::cout << "calcVol: " << calcVol*1000000 << std::endl;

		volume = calcVol * 1000000;
	}

	//plane model segmentation
	bool planeModelSegmentation(pcl::PointCloud<pclPoint>::Ptr cloud, pcl::PointCloud<pclPoint> & cloud_f, pcl::PointCloud<pclPoint> & cloud_p, double th = 0.02) {
		pcl::SACSegmentation<pclPoint> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(th);
		cloud_f = *cloud;

		int i = 0, nr_points = (int)cloud_f.points.size();
		bool succes = false;
		while (cloud_f.points.size() > 0.3 * nr_points)
		{
			if (succes) return false;
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(cloud);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pclPoint> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);

			// Get the points associated with the planar surface
			extract.filter(cloud_p);
			//std::cout << "PointCloud representing the planar component: " << cloud_p.points.size() << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			extract.setNegative(true);
			extract.filter(cloud_f);
			//*cloud = *cloud_f;

			/*pcl::PointCloud<pclPoint>::Ptr viewcloud(new pcl::PointCloud<pclPoint>);
			*viewcloud = cloud_f;

			pcl::visualization::CloudViewer viewer("Filter viewer");
			viewer.showCloud(viewcloud);
			while (!viewer.wasStopped())
			{

			}*/
			succes = true;
		}
		return true;
	}

	//statical outlier removel filter
	vector<int> statisticalOutlierRemovel(pcl::PointCloud<pclPoint>::Ptr cloud) {
		pcl::StatisticalOutlierRemoval<pclPoint> sor;
		vector<int> outliers;
		sor.setInputCloud(cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(2.5);
		//std::cout << "Start statisticalOutlierRemovel filter" << std::endl;
		sor.filter(outliers);

		pcl::PointCloud<pclPoint>::Ptr newCloud(new pcl::PointCloud<pclPoint>);
		newCloud->width = outliers.size();
		newCloud->height = 1;
		newCloud->resize(outliers.size());

		for (int i = 0; i < outliers.size(); i++) {
			newCloud->points.at(i) = cloud->points.at(outliers.at(i));
		}

		*cloud = *newCloud;

		//std::cout << "Finish statisticalOutlierRemovel filter" << std::endl;
		return outliers;
	}

	pcl::PointCloud<pclPoint>::Ptr calculatePointPosition(rs2::points points){
		return pointsToPCL(points);
	}

	/*
	Mein Teil
	*/

	double getClusterCenter(pcl::PointIndices cluster, pcl::PointCloud<pclPoint>::Ptr cloud){
    	//pcl::PointCloud<pclPoint>::Ptr cloud(new pcl::PointCloud<pclPoint>);
		Eigen::Vector4f centroid;
 		//pcl::compute3DCentroid(*cloud,centroid);
 		//cout<<new_centroid[0] << endl << new_centroid[1] << endl << new_centroid[2] << endl; 

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		inliers->indices = cluster.indices;

		pcl::ExtractIndices<pclPoint> extract;
		// Extract the inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*cloud);
		//showCloud(cloud, "test");
		double mean = 0;

		for(auto& p : cloud->points){
			mean += p.x;
			mean += p.y;
		}
		return mean / cloud->points.size();
	}

	//Color based region Growing Segmentation
	pcl::PointCloud<pclPoint>::Ptr colorBasedRegionGrowing(pcl::PointCloud<pclPoint>::Ptr cloud,
															int distanceThreshold,
															int pointColorThreshold,
															int regionColorThreshold, 
															bool getNegative = false, 
															bool showCBRGcloud = false, 
															bool showExtractedCloud = false,
															bool smaller = false){

		pcl::RegionGrowingRGB<pclPoint> reg;
		pcl::search::Search <pclPoint>::Ptr tree (new pcl::search::KdTree<pclPoint>);

		pcl::IndicesPtr indices(new std::vector <int>);
		pcl::PassThrough<pclPoint> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 1.0);
		pass.filter(*indices);
		
		reg.setInputCloud(cloud);
		reg.setIndices(indices);
		reg.setSearchMethod(tree);
		reg.setDistanceThreshold(distanceThreshold);
		reg.setPointColorThreshold(pointColorThreshold);
		reg.setRegionColorThreshold(regionColorThreshold);
		reg.setMinClusterSize(600);

		std::vector<pcl::PointIndices> clusters;    
		std::vector<double> centerPoints;
		reg.extract(clusters);
		int clusterSize = 0;
		int clusterNr = 0;

		for(int i = 0; i < clusters.size(); i++){
			if(smaller){
				//cout << getClusterCenter(clusters[i], cloud) << endl;
			}
			if(clusterSize < clusters[i].indices.size()){
				clusterSize = clusters[i].indices.size();
				clusterNr = i;
			}
			//cout << i <<": " << clusters[i].indices.size() << endl;
		}

		pcl::PointCloud <pclPoint>::Ptr colored_cloud = reg.getColoredCloud();
		if(showCBRGcloud){
			pclfuncs::showCloud(colored_cloud, "color based region growing");
		}


		if(clusters[clusterNr].indices.size() > cloud->points.size()* 0.1){
			
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			inliers->indices = clusters[clusterNr].indices;
			pcl::ExtractIndices<pclPoint> extract;
			// Extract the inliers
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(getNegative);
			extract.filter(*cloud);

			if(showExtractedCloud){
				pclfuncs::showCloud(cloud, "exctracted cloud");
			}
			return cloud;
		}    
    	return cloud;    
	}
}

#endif