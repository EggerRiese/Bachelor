import h5py
import os, os.path
import numpy as np
from plyfile import PlyData, PlyElement
import glob
import open3d as o3d
from pathlib import Path

NUM_FRAMES = 15

data = np.zeros((NUM_FRAMES, 4096, 6), dtype = np.float32)
label = np.zeros((NUM_FRAMES,  4096),dtype = np.uint8)
 	
f = h5py.File('real_data.h5', 'w')


home = str(Path.home())
print(home)
labeled_data_dir = '/home/joshua/Dokumente/Bachelor/Aufnahmen/dataset/POINTNET_LABELED_REAL_DATA/'
rgb_data_dir = '/home/joshua/Dokumente/Bachelor/Aufnahmen/dataset/POINTNET_REAL_DATA/'
i = -1

for filepath in glob.iglob(labeled_data_dir+'*.txt'):
    if(i == NUM_FRAMES-1):
        break; 
    i = i+1
    #xyz_label_ply = PlyData.read(filepath)
    #xyz_label_ply = o3d.io.read_point_cloud(filepath)
    file_obj = open(filepath)   
    filepath = os.path.relpath(filepath,labeled_data_dir)
    file_no = filepath[:-4]
 
    
    labels = np.zeros((4096, 1))
    xyz = np.zeros((4096, 3))
    colors = np.zeros((4096, 3))
    
    rgb_pcd_path = rgb_data_dir + file_no + '.pcd'
    rgb_pcd = o3d.io.read_point_cloud(rgb_pcd_path)
    xyz = np.asarray(rgb_pcd.points)
    colors = np.asarray(rgb_pcd.colors)
    labels = file_obj.read().split("\n")
    
    #print(len(labels))
    #print(colors[1][1])
    #print(xyz_label_ply.elements)
    #print(xyz_label_ply['vertex']['label'][1])
    
    for j in range(0, 4096):

       data[i, j] = [xyz[j][0], xyz[j][1], xyz[j][2], colors[j,0], colors[j,1],colors[j,2]]  
       #data[i, j] = [xyz_label_ply['vertex']['x'][j], xyz_label_ply['vertex']['y'][j], xyz_label_ply['vertex']['z'][j], colors[j,0], colors[j,1],colors[j,2]]           
       #print(xyz_label_ply['vertex']['object'][j])
       label[i,j] = labels[j]

f.create_dataset('data', data = data)
f.create_dataset('label', data = label)