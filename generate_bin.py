# -*- coding: utf-8 -*-
from __future__ import division
import os
import numpy as np
import mayavi.mlab as mlab
from math import cos,sin
import pandas as pd
from tqdm import tqdm

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from kitti_util import *


BASE_PATH = "/media/seuiv/ae1d4362-67a6-4fd5-9b1b-83abf391090e/Data/kitti/training/"


def draw_lidar(lidar, is_grid=False, is_top_region=True, fig=None):
    pxs=lidar[:,0]
    pys=lidar[:,1]
    pzs=lidar[:,2]
    prs=lidar[:,3]
    if fig is None: fig = mlab.figure(figure=None, bgcolor=(1,1,1), fgcolor=None, engine=None, size=(1000, 500))
    mlab.points3d(
        pxs, pys, pzs, prs,
        mode='point',
        colormap='spectral',
        figure=fig)
 
def draw_gt_boxes3d(gt_boxes3d, fig, color=(1,0,0), line_width=1):
    num = len(gt_boxes3d)
    for n in range(num):
        b = gt_boxes3d[n]
        for k in range(0,4):
            i,j=k,(k+1)%4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
            i,j=k+4,(k+3)%4 + 4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
            i,j=k,k+4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
    mlab.view(azimuth=180,elevation=None,distance=50,focalpoint=[12.0909996 , -1.04700089, -2.03249991])
 


def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    # 计算旋转矩阵
    R = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
    # 8个顶点的xyz
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h] 
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    # 旋转矩阵点乘(3，8)顶点矩阵
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    # 加上location中心点，得出8个顶点旋转后的坐标
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2



def load_kitti_label(label_path, calib_path, firstname, lidar):

    LABEL_NAME = ["type", "truncated", 
    "occluded", "alpha", 
    "bbox_left", "bbox_top", "bbox_right", "bbox_bottom", 
    "dimensions_height", "dimensions_width", "dimensions_length", 
    "location_x", "location_y", "location_z", "rotation_y"] 
    df = pd.read_csv(label_path, sep=' ', header=None, index_col=None)
    df.columns = LABEL_NAME
    df.loc[df.type.isin(['Van','Car','Truck']),'type'] = 'Car'
    df = df[df.type.isin(['Car'])]
 
    n = len(df)
    box_points = []
    calib = Calibration(calib_path)
 
    for i in range(n):      
        corners_3d_cam2 = compute_3d_box_cam2(*df.iloc[ i ,[8, 9, 10, 11, 12, 13, 14]])
        # cam2转velo坐标系
        corners_3d_velo = calib.project_rect_to_velo(corners_3d_cam2.T)

        xyz_min = np.min(corners_3d_velo, axis=0) # x1 y1 z1
        xyz_max = np.max(corners_3d_velo, axis=0) # x2 y2 z2

        valid = (lidar[:, 0] > xyz_min[0]) & (lidar[:, 0] < xyz_max[0]) & (lidar[:, 1] > xyz_min[1]) & (lidar[:, 1] < xyz_max[1]) & (lidar[:, 2] > xyz_min[2]) & (lidar[:, 2] < xyz_max[2])
        temp_lidar = lidar[valid]
        temp_lidar_name = './pcds/' + firstname + '_' + str(i) +'.bin'
        temp_lidar.tofile(temp_lidar_name)

        txt_name = './boxes/' + firstname + '_' + str(i) +'.txt'
        np.savetxt(txt_name,corners_3d_velo,delimiter=' ')
        box_points.append(corners_3d_velo)

    return np.array(box_points)

if __name__ == '__main__':
    lidar_path = os.path.join(BASE_PATH, "velodyne/")
    label_path = os.path.join(BASE_PATH, "label_2/")
    calib_path = os.path.join(BASE_PATH, "calib/")
    for lidars in tqdm(os.listdir(lidar_path)):
        lidar_file = lidar_path + str(lidars)
        firstname,lastname = lidars.split('.')
        labels = firstname + '.txt'

        pointcloud = np.fromfile(lidar_file, dtype=np.float32, count=-1).reshape([-1, 4])
        # print(pointcloud.shape)

        if labels in os.listdir(label_path):
            label_file = label_path + str(labels)
            calib_file = calib_path + str(labels)
 
            lidar = np.fromfile(lidar_file, dtype=np.float32)
            lidar = lidar.reshape((-1, 4))

            # print(firstname)
            gt_box3d = load_kitti_label(label_file, calib_file, firstname, lidar)

            # fig = draw_lidar(lidar, is_grid=True, is_top_region=True)
            # draw_gt_boxes3d(gt_boxes3d=gt_box3d, fig=fig)
            # mlab.show()
    

    # lidar_file = os.path.join(BASE_PATH, "velodyne/000008.bin")
    # label_file = os.path.join(BASE_PATH, "label_2/000008.txt")
    # calib_file = os.path.join(BASE_PATH, "calib/000008.txt")

    # lidar = np.fromfile(lidar_file, dtype=np.float32)
    # lidar = lidar.reshape((-1, 4))
    # firstname = '000008'
    # gt_box3d = load_kitti_label(label_file, calib_file, firstname, lidar)
    
    # # retvalid = [False] * lidar.shape[0]
    # # for i in gt_box3d:
    # #     xyz_min = np.min(i, axis=0) # x1 y1 z1
    # #     xyz_max = np.max(i, axis=0) # x2 y2 z2

    # #     valid = (lidar[:, 0] > xyz_min[0]) & (lidar[:, 0] < xyz_max[0]) & (lidar[:, 1] > xyz_min[1]) & (lidar[:, 1] < xyz_max[1]) & (lidar[:, 2] > xyz_min[2]) & (lidar[:, 2] < xyz_max[2])
    # #     retvalid = retvalid | valid

    # # lidar = lidar[retvalid]
    # # lidar.tofile('00008.bin')

    
    # fig = draw_lidar(lidar, is_grid=True, is_top_region=True)
    # draw_gt_boxes3d(gt_boxes3d=gt_box3d, fig=fig)
    # mlab.show()