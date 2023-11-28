#!/usr/bin/env python3
  
import rospy
import cv2
import os,rospkg
import numpy as np
import math
import time
import rospkg
import os 
import json
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pcs
from sensor_msgs.msg import Image, CompressedImage
from morai_msgs.msg import GPSMessage
from cv_bridge import CvBridgeError
from cv_bridge import CvBridge 
from slidewindow import SlideWindow
from morai_msgs.msg import CtrlCmd
import tf
from std_msgs.msg import Float64,Int16,Float32MultiArray
from nav_msgs.msg import Path,Odometry


class IMGParser3:
    def __init__(self, params_cam, params_lidar):
        
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.RT = self.transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx =  self.project2ing_mtx(params_cam)


    def transform_lidar2cam(self, xyz_p):
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis = 1), self.RT.T)

        return xyz_c       


    #좌표 변환 행렬 정의
    def translationMtx(self,x, y, z):
        M = np.array([[1,   0,  0,  x],
                      [0,   1,  0,  y],
                      [0,   0,  1,  z],
                      [0,   0,  0,  1],
                      ])
        return M
    
    #좌표 변환 행렬 정의
    def rotationMtx(self,yaw, pitch, roll):
        R_x = np.array([[1,     0,              0,                  0],
                        [0,     math.cos(roll), -math.sin(roll),    0],
                        [0,     math.sin(roll),  math.cos(roll),    0],
                        [0,     0,              0,                  1], 
                        ])
        
        R_y = np.array([[math.cos(pitch),    0,  math.sin(pitch),     0],
                        [0,                 1,  0,                  0],
                        [-math.sin(pitch),   0,  math.cos(pitch),     0],
                        [0,                 0,  0,                  1], 
                        ])
        
        R_z = np.array([[math.cos(yaw),     -math.sin(yaw),  0,  0],
                        [math.sin(yaw),     math.cos(yaw),  0,  0],
                        [0,                 0,              0,  0],
                        [0,                 0,              0,  1], 
                        ])
        R = np.matmul(R_x, np.matmul(R_y, R_z))

        return R
        
    #Lidar2cam 좌표 변환 생성 
    def transformMTX_lidar2cam(self, params_lidar, params_cam):

        lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
        cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

        x_rel = cam_pos[0] - lidar_pos[0]
        y_rel = cam_pos[1] - lidar_pos[1]
        z_rel = cam_pos[2] - lidar_pos[2]

        R_T = np.matmul(self.translationMtx(x_rel, y_rel, z_rel), self.rotationMtx(np.deg2rad(-90.),0.,0.))
        R_T = np.matmul(R_T, self.rotationMtx(0,  0.,   np.deg2rad(-90.)))
        

        R_T = np.linalg.inv(R_T)

        return R_T        
    
    def project2img_mtx(self, params_cam):
        fc_x = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["HEIGHT"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))


        cx = params_cam["WIDTH"] / 2
        cy = params_cam["HEIGHT"] / 2

        R_f = np.array([[fc_x, 0,    cx],
                        [0,    fc_y, cy]])

        return R_f

    def draw_pts_img(img, xi, yi):
        point_np = img

        for ctr in zip(xi, yi):
            point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

        return point_np
    
    def project_pts2img(self, xyz_c, crop=True):
        
        xyz_c = xyz_c.T

        xc, yc, zc = xyz_c[0, :].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])

        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)

        xyi = np.matmul(self.proj_mtx, np.concatenate([xn,yn, np.ones_like(xn)], aixs = 0))

        xyi = xyi[0:2,:].T

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass

        return xyi
    
    def crop_pts(self, xyi):
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi


if __name__ == '__main__':
    rospy.init_node('image_parser',anonymous=True)

    image_parser = IMGParser3()

    rospy.spin()