import sys
import numpy as np
#np.set_printoptions(threshold=sys.maxsize)
import open3d 
import open3d as o3d
import glob
import os, os.path
import h5py

obj_path = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/PointNetSegmentation/"
i = 0

for filepath in glob.iglob(obj_path+'*_gt.obj'):
    mesh = o3d.io.read_triangle_mesh(filepath)
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.colors = mesh.vertex_colors
    pcd.normals = mesh.vertex_normals
    o3d.io.write_point_cloud("/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/PointNetSegmentation/" + str(i) + "_gt.pcd", pcd)
    i += 1
    o3d.visualization.draw_geometries([pcd])
