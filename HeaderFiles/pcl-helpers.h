#pragma once

#ifndef PCL_HELPERS_H_
#define PCL_HELPERS_H_

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <BaseHeader.h>


//constant values to cut the point cloud (only a section is needed, reduce amount of data and speed up calculations)
const double portion_cut_out_top = 0.20, portion_cut_out_bottom = 0.20, portion_cut_out_left = 0.30, portion_cut_out_right = 0.30;
//const double portion_cut_out_top = 0, portion_cut_out_bottom = 0, portion_cut_out_left = 0, portion_cut_out_right = 0;


std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, float TextureU, float TextureV)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(TextureU * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(TextureV * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//Convert Intel point cloud to PCL and cut at the edges with above defined margins
pcl::PointCloud<pclPoint>::Ptr pointsToPCL(rs2::points points) {
	//Resize Pointcloud-Window in portion

	pcl::PointCloud<pclPoint>::Ptr cloud(new pcl::PointCloud<pclPoint>);
	
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	
	//precalculate values
	int old_width = sp.width(),
		old_height = sp.height(),
		cut_out_top = old_height * portion_cut_out_top,
		cut_out_bottom = old_height * portion_cut_out_bottom,
		cut_out_left = old_width * portion_cut_out_left,
		cut_out_right = old_width * portion_cut_out_right,
		cut_out_width = cut_out_left + cut_out_right,
		cut_out_height = cut_out_top + cut_out_bottom,
		new_width = old_width - cut_out_width,
		new_height = old_height - cut_out_height;
	cloud->width = new_width;
	cloud->height = new_height;
	cloud->is_dense = true; //false
	cloud->points.resize(new_width*new_height);
	auto ptr = points.get_vertices();

	//Set ptr to the new top left corner with cutting out top and left from old height and width
	ptr += (cut_out_top*old_width + cut_out_left);
	int counter_points = 0;
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;

		//Adding offset of left and right cutout to ptr of "old" rs2 pointcloud
		if (counter_points % new_width == new_width - 1) {
			ptr += (cut_out_right + cut_out_left);
		}

#ifdef PCL_RGB
		p.r = 255;
		p.g = 255;
		p.b = 255;
#endif

		ptr++;
		counter_points++;
	}

	return cloud;
}


//Calculate PCL point cloud from intel Frame (first intel point cloud, after that convert to pcl)
pcl::PointCloud<pclPoint>::Ptr calculatePointcloud(rs2::frame frame)
{
	rs2::depth_frame depth = frame.as<rs2::depth_frame>();
	//new
	//auto RGB = frame.get_color_frame();
	rs2::pointcloud pc;
	rs2::points points = pc.calculate(depth);
	// new
    //pc.map_to(RGB);
	
	return pointsToPCL(points);
}



//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
pcl::PointCloud<pclPoint>::Ptr PCL_Conversion(rs2::points points,  rs2::video_frame color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pclPoint>::Ptr cloud(new pcl::PointCloud<pclPoint>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

	int old_width = sp.width(),
		old_height = sp.height(),cut_out_top = old_height * portion_cut_out_top,
		cut_out_bottom = old_height * portion_cut_out_bottom,
		cut_out_left = old_width * portion_cut_out_left,
		cut_out_right = old_width * portion_cut_out_right,
		cut_out_width = cut_out_left + cut_out_right,
		cut_out_height = cut_out_top + cut_out_bottom,
		new_width = old_width - cut_out_width,
		new_height = old_height - cut_out_height;

    cloud->width  = new_width;   
    cloud->height = new_height;
    cloud->is_dense = false;
    cloud->points.resize( new_width * new_height);
	
    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();
	
	//Set ptr to the new top left corner with cutting out top and left from old height and width
	Vertex += (cut_out_top*old_width + cut_out_left);
	Texture_Coord += (cut_out_top*old_width + cut_out_left);

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
	int counter = 0;
    for (auto& p : cloud->points)
    {   
		
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        p.x = Vertex->x;
        p.y = Vertex->y;
        p.z = Vertex->z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord->u, Texture_Coord->v);

        // Mapping Color (BGR due to Camera Model)
        p.r = std::get<0>(RGB_Color); // Reference tuple<2>
        p.g = std::get<1>(RGB_Color); // Reference tuple<1>
        p.b = std::get<2>(RGB_Color); // Reference tuple<0>

		//Adding offset of left and right cutout to ptr of "old" rs2 pointcloud
		if (counter % new_width == new_width - 1) {
			Vertex += (cut_out_right + cut_out_left);
			Texture_Coord += (cut_out_right + cut_out_left);
		}
		Vertex++;
		Texture_Coord++;
		counter++;
    }
    
   return cloud; // PCL RGB Point Cloud generated
}

//Calculate PCL point cloud from intel Frame (first intel point cloud, after that convert to pcl)
pcl::PointCloud<pclPoint>::Ptr calculateColorPointcloud(rs2::frameset frame)
{
    auto depth = frame.get_depth_frame();
    auto RGB = frame.get_color_frame();
	rs2::pointcloud pc;
	// Map Color texture to each point
    pc.map_to(RGB);
	
    // Generate Point Cloud
    auto points = pc.calculate(depth);
	return PCL_Conversion(points, RGB);
}

#endif