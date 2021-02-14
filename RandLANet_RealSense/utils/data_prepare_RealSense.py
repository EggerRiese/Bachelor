from sklearn.neighbors import KDTree
from os.path import join, exists, dirname, abspath
from pypcd import pypcd 
import numpy as np
import pandas as pd
import os, sys, glob, pickle
import open3d as o3d

BASE_DIR = dirname(abspath(__file__))
ROOT_DIR = dirname(BASE_DIR)
sys.path.append(BASE_DIR)
sys.path.append(ROOT_DIR)
from helper_ply import write_ply
from helper_tool import DataProcessing as DP

dataset_path = '/home/joshua/Dokumente/Bachelor/RandLANet_RealSense/data/RealSense/'

sub_grid_size = 0.010
original_pc_folder = join(dirname(dataset_path), 'original_ply')
print(join(dirname(dataset_path), 'original_ply'))
sub_pc_folder = join(dirname(dataset_path), 'input_{:.3f}'.format(sub_grid_size))
os.mkdir(original_pc_folder) if not exists(original_pc_folder) else None
os.mkdir(sub_pc_folder) if not exists(sub_pc_folder) else None
out_format = '.ply'


def convert_pc2ply(filePath, save_path):
    """
    Convert original dataset files to ply file (each line is XYZRGBL).
    We aggregated all the points from each instance in the room.
    :param anno_path: path to annotations. e.g. Area_1/office_2/Annotations/
    :param save_path: path to save original point clouds (each line is XYZRGBL)
    :return: None
    """

    cloud = pypcd.PointCloud.from_path(filePath)

    # convert the structured numpy array to a ndarray
    new_cloud_data = cloud.pc_data.view(np.float32).reshape(cloud.pc_data.shape + (-1,))

    # get the labels
    labels = cloud.pc_data['label']

    # split the rgb column into three columns: red, green and blue
    rgb_columns = pypcd.decode_rgb_from_pcl(cloud.pc_data['rgb'])

    new_cloud_data = np.delete(new_cloud_data, [3, 4, 5], axis=1)
    

    xyz = new_cloud_data.astype(np.float32)
    colors = rgb_columns.astype(np.uint8)
    labels = labels.astype(np.uint8)
    write_ply(save_path, (xyz, colors, labels), ['x', 'y', 'z', 'red', 'green', 'blue', 'class'])

    # save sub_cloud and KDTree file
    sub_xyz, sub_colors, sub_labels = DP.grid_sub_sampling(xyz, colors, labels, sub_grid_size)
    sub_ply_file = join(sub_pc_folder, save_path.split('/')[-1][:-4] + '.ply')
    write_ply(sub_ply_file, [sub_xyz, sub_colors, sub_labels], ['x', 'y', 'z', 'red', 'green', 'blue', 'class'])

    search_tree = KDTree(sub_xyz)
    kd_tree_file = join(sub_pc_folder, str(save_path.split('/')[-1][:-4]) + '_KDTree.pkl')
    with open(kd_tree_file, 'wb') as f:
        pickle.dump(search_tree, f)

    proj_idx = np.squeeze(search_tree.query(xyz, return_distance=False))
    proj_idx = proj_idx.astype(np.int32)
    proj_save = join(sub_pc_folder, str(save_path.split('/')[-1][:-4]) + '_proj.pkl')
    with open(proj_save, 'wb') as f:
        pickle.dump([proj_idx, labels], f)

    
    #print(save_path)
    #cloud = o3d.io.read_point_cloud("/home/joshua/Dokumente/Bachelor/RandLa-Net/RandLA-Net/data/RealSense/input_0.010/Area_1_kaese_1.ply") # Read the point cloud
    #o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud  


if __name__ == '__main__':
    dirPath = "/home/joshua/Dokumente/Bachelor/Aufnahmen/Studie/RandLaNet/evaluation/labeled/"
    for filePath in glob.iglob(dirPath + '*.pcd'):
        print(filePath)
        elements = str(filePath).split('/')
        name = str(elements[-1]).split('.')
        out_file_name = str(name[0]) + out_format
        convert_pc2ply(filePath, join(original_pc_folder, out_file_name))
