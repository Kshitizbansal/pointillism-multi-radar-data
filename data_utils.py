

import os
import numpy as np
import json
import pandas as pd
import matplotlib.pyplot as plt
import open3d as o3d
class Data:
    def __init__(self, dataset):
        # Gets all required data
        data_folder = "./data/scene{0}/".format(dataset.dataset[0])
        self.camera_folder = data_folder + "images/"
        self.label_folder = data_folder + "label/"
        self.lidar_folder = data_folder + "lidar/"

        self.radar_folder = data_folder + "radar_{0}/".format(dataset.radar[0])
        self.camera = sorted(os.listdir(self.camera_folder))
        self.label = sorted(os.listdir(self.label_folder))
        self.lidar = sorted(os.listdir(self.lidar_folder))
        self.radar = sorted(os.listdir(self.radar_folder))

        self.dataset = dataset
    
    def get_radar(self, file_name):
        listdata = pd.read_csv(self.radar_folder+file_name, delimiter=",")
        listdata = np.array(listdata.values[:,[0,1,4,3,2,5,6,7,8,9]])
        if self.dataset.radar[0] == 0:
            listdata[:,2]-=0.5
        else:
            listdata[:,2]+=0.5
        return listdata
    
    def get_lidar(self, file_name):
        pcd = o3d.io.read_point_cloud(self.lidar_folder+file_name)
        points = []
        for point in pcd.points:
            if not (point[0] == point[1] == point[2]):
                points.append([point[1], point[2], point[0]])
        return np.asarray(points)
    
    def get_label(self, file_name):
        file_read = open(self.label_folder+file_name)
        label = json.load(file_read)
        file_read.close()
        box = []
        for item in label['labels']:
            center_x = item['center']['x']
            center_y = item['center']['y']
            center_z = item['center']['z']
            d_x = item['size']['x']
            d_y = item['size']['y']
            d_z = item['size']['z']
            theta = -item['orientation']['z']
            box.append([d_x,d_y,d_z,center_x,center_y,center_z,theta])
        return box

    def get_camera(self, file_name):
        return plt.imread(self.camera_folder+file_name)

    def read_data(self, idx):
        radardata = self.get_radar(self.radar[idx])
        im = self.get_camera(self.camera[idx])
        lidardata = self.get_lidar(self.lidar[idx])
        label_box = self.get_label(self.label[idx])
        return im, radardata, lidardata, label_box
    
    def lidar2CameraOurs(self, lidar_pc):
        ''' 
        This is the projection code for our dataset to project pointcloud onto the camera plane for the mask based clustering
        '''

        proj = np.array([[320, 640, 0, -96],
                         [240, 0, 640, -72],
                         [1, 0, 0, -0.3]]).reshape(3,4)
        image_coords = np.zeros((lidar_pc.shape[0],2))
        for pidx, points in enumerate(lidar_pc):   
            point = points[:3]
            point = np.array([[point[0], point[1], -point[2], 1]]).reshape(4,1)
            point = np.matmul(proj, point)
            pixel = [640 - point[0]/point[2], 480 - point[1]/point[2]]
            image_coords[pidx] = pixel
        return image_coords
    