
from data_utils import Data
import numpy as np
import os
from matplotlib import pyplot as plt
import open3d as o3d

saveDir = "./dataImage/"
vid_path = "./video/"
class Process:
    def __init__(self, args):
        self.args = args
        self.data = Data(args)
        self.choice = {
            'im'    : None,
            'type'  : None,
            'box'   : [],
            'points': None,
            'video': args.video,
            'index': None,
        } 
    
    def calculate_box(self, choice): # helper that add each points for each item
        boxes = []
        for box_gt in choice:
            boxes.append(self.groundTruths(box_gt))
        return boxes

    def groundTruths(self, box): # Give plot position given whlxyza and returns 8 points of bbox
        
        width_fromOrigin = box[0] / 2 # X 
        length_fromOrigin = box[1] / 2 # Y
        height_fromOrigin = box[2] / 2 # Z
        x_distance = box[3]
        y_distance = box[4]
        z_distance = box[5]
        yaw_angle = box[6] # Yaw Angle
        
        # Give dimensions to box
        box_points = []
        box_points.append([ [np.negative(width_fromOrigin)], [np.negative(length_fromOrigin)], [np.negative(height_fromOrigin)] ]) #1
        box_points.append([ [width_fromOrigin],              [np.negative(length_fromOrigin)], [np.negative(height_fromOrigin)] ]) #2
        box_points.append([ [np.negative(width_fromOrigin)], [length_fromOrigin],              [np.negative(height_fromOrigin)] ]) #3
        box_points.append([ [width_fromOrigin],              [length_fromOrigin],              [np.negative(height_fromOrigin)] ]) #4

        box_points.append([ [np.negative(width_fromOrigin)], [np.negative(length_fromOrigin)], [height_fromOrigin] ]) #5
        box_points.append([ [width_fromOrigin],              [np.negative(length_fromOrigin)], [height_fromOrigin] ]) #6
        box_points.append([ [np.negative(width_fromOrigin)], [length_fromOrigin],              [height_fromOrigin] ]) #7
        box_points.append([ [width_fromOrigin],              [length_fromOrigin],              [height_fromOrigin] ]) #8
        box_points = np.reshape(box_points, (len(box_points), 3))
        
        # Rotate Box
        yaw_matrix = np.matrix([ 
                        [np.cos(yaw_angle), np.negative(np.sin(yaw_angle)), 0], 
                        [np.sin(yaw_angle), np.cos(yaw_angle), 0],
                        [0, 0, 1]
                                                                                ])
        
        for item in range(8):
            x = np.matmul(yaw_matrix, np.reshape(box_points[item], (3,1)))
            box_points[item] = np.reshape(x, (1,3))
            # Translate the box
            box_points[item][0]+=x_distance
            box_points[item][1]+=y_distance
            box_points[item][2]+=z_distance

        return box_points.tolist()

    def saveImageOurs(self, choice):
        image = []
        for item in choice['box']:
            image.append(self.data.lidar2CameraOurs(np.asarray(item)))
        choice['box'] = image

    def plotBird(self, choice): # Plot 2d Radar pointbox
        
        plt.scatter(choice['points'][:,0], choice['points'][:,2], c='g', s=1)
        
        for item in choice['box']:
            plt.plot((item[0][1], item[2][1]), (item[0][0], item[2][0]), color="red", linewidth=0.5)
            plt.plot((item[1][1], item[3][1]), (item[1][0],item[3][0]), color="red", linewidth=0.5)
            for i in range(0,4,2):
                plt.plot((item[i][1], item[i+1][1]), (item[i][0],item[i+1][0]), color="red", linewidth=0.5)

        # Set your view
        plt.xlim([-20, 20])
        plt.ylim([-30, 10])
        plt.savefig(saveDir+"{0}.jpg".format(str(format(choice['index'], '06d'))), bbox_inches='tight')
        if not choice['video']:
            plt.show()
        plt.close()
        

    def saveImage(self, plt, choice): # Used to save image and make cleaner program
        plt.tight_layout()
        plt.imshow(choice['rgb'])
        plt.savefig(saveDir+"{0}.jpg".format(str(format(choice['index'], '06d'))), bbox_inches='tight')
        if not choice['video']:
            plt.show()
        plt.close()

    def drawGTBox(self, box, plt): # Func to draw GT box
        for item in box:
            plt.plot((item[0][0], item[2][0]), (item[0][1],item[2][1]), color="blue", linewidth=1)
            plt.plot((item[4][0], item[6][0]), (item[4][1],item[6][1]), color="blue", linewidth=1)
            plt.plot((item[0][0], item[4][0]), (item[0][1],item[4][1]), color="blue", linewidth=1)
            plt.plot((item[2][0], item[6][0]), (item[2][1],item[6][1]), color="blue", linewidth=1)

            plt.plot((item[1][0], item[3][0]), (item[1][1],item[3][1]), color="blue", linewidth=1)
            plt.plot((item[1][0], item[5][0]), (item[1][1],item[5][1]), color="blue", linewidth=1)
            plt.plot((item[3][0], item[7][0]), (item[3][1],item[7][1]), color="blue", linewidth=1)
            plt.plot((item[5][0], item[7][0]), (item[5][1],item[7][1]), color="blue", linewidth=1)

            for i in range(0,7,2):
                plt.plot((item[i][0], item[i+1][0]), (item[i][1],item[i+1][1]), color="blue", linewidth=1)

         
    def drawPointCloud(self, choice): # Plot pointclouds for Radar and Lidar 
        plt.scatter(choice['points'][:,1], choice['points'][:,0], s=1, color='red')
        self.drawGTBox(choice['box'], plt)
        self.saveImage(plt, choice)

    def drawImageMatplot(self, choice): # Plot Camera for gt or predicted
        self.drawGTBox(choice['box'], plt)
        self.saveImage(plt, choice)
    
    def plotOpen3D(self, choice): # Plot for both Lidar and Radar in 3D

        # Plot all the point cloud and assign doppler coloring
        x = o3d.geometry.PointCloud()
        x.points = o3d.utility.Vector3dVector(choice['points'])
        # Draw all the bounding boxes
        line_set = []
        for points in choice['box']:  
            lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
                    [0, 4], [1, 5], [2, 6], [3, 7]]
            colors = [[1, 0, 0] for i in range(len(lines))]
            line = o3d.geometry.LineSet()
            line.points = o3d.utility.Vector3dVector(points)
            line.lines = o3d.utility.Vector2iVector(lines)
            line.colors = o3d.utility.Vector3dVector(colors)
            try:
                line_set = [line + line_set[0]]
            except IndexError:
                line_set = [line]

        # Create Visualizer and add point cloud and point clouds
        vis = o3d.visualization.Visualizer()
        if not choice['video']:
            vis.create_window()
        else:
            vis.create_window(visible=False)
        vis.get_render_option().background_color = np.asarray([0, 0, 0])
        if line_set:
            vis.add_geometry(line_set[0])
        vis.add_geometry(x)
        vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(5, [0,0,0]))

        '''
        # Set View, Change View params depending on frames
        ctr = vis.get_view_control()
        ctr.set_zoom(0.21999999999999947)
        ctr.set_front([ 0.024911310794565964, -0.9871957638433011, 0.15755618186646272 ])
        ctr.set_lookat([ 14.750686164733382, 13.089020076195469, -25.389724294591712 ])
        ctr.set_up([ 0.019932380474399262, -0.15806424970107846, -0.98722762986813684  ])''
        '''
        # View if there is no video
        if not choice['video']:
            vis.run()
            vis.destroy_window()
        
        # Save and update shapes if there is video
        else:  
            vis.update_geometry(x)
            if line_set:
                vis.update_geometry(line_set[0])
            vis.poll_events()
            vis.update_renderer()
            vis.capture_screen_image(saveDir+"{0}.jpg".format(str(format(choice['index'], '06d'))))
            vis.destroy_window()

    def storeChoice(self, index): # Stores user Choices
        im, radardata, lidardata, label_box = self.data.read_data(index)
        self.choice['index'] = index

        if self.choice['type'] == "Lidar":
            self.choice['box'] = self.calculate_box(label_box)
            self.choice['points'] = lidardata[lidardata[:,2]<0]
            if self.args.type == "Lidar3D":
                self.choice['points'] = self.choice['points'][:,[2,0,1]]

        elif self.choice['type'] == "Radar":
            self.choice['box'] = self.calculate_box(label_box)
            self.choice['points'] = radardata[:,2:5]
            self.choice['points'][:,2] = np.negative(radardata[:,4])
            self.choice['points'] = self.choice['points'][self.choice['points'][:,0]<0]
            if self.args.type == "Radar3D":
                self.choice['points'] = self.choice['points'][:,[2,0,1]]

        elif self.choice['type'] == "Camera":
            self.choice['rgb'] = im
            self.choice['box'] = self.calculate_box(label_box)

            
    def Lidar(self): # Go to Lidar func and parse choices
        self.choice['type'] = "Lidar"

        for index in range(self.args.frame[0], self.args.frame[1]):
            self.storeChoice(index)
            
            if self.args.type == "Lidar3D":
                self.plotOpen3D(self.choice)

            elif self.args.type == "LidarBird":
                self.plotBird(self.choice)

    def Radar(self): # Go to Radar func and parse choices
        self.choice['type'] = "Radar"
        for index in range(self.args.frame[0], self.args.frame[1]):
            self.storeChoice(index)
            
            if self.args.type == "Radar3D": 
                self.plotOpen3D(self.choice)
                
            elif self.args.type == "RadarBird":
                self.plotBird(self.choice)


    def Camera(self): # Go to Camera func and parse choices
        self.choice['type'] = 'Camera'

        for index in range(self.args.frame[0], self.args.frame[1]):
            self.storeChoice(index)
            self.saveImageOurs(self.choice)
            self.drawImageMatplot(self.choice)


    def saveVideo(self): # Save images to videos
        image_path = saveDir
        video_path = vid_path + '{0}.mp4'.format(self.args.dataset)
        os.system('ffmpeg -i {1}{2} -vf "setpts=5*PTS,pad=ceil(iw/2)*2:ceil(ih/2)*2" {0} '.format(video_path, image_path, '%06d.jpg'))
        
    def options(self): # Goes to specific function with given 
        self.checkdir()
        if self.args.type[0:5] == "Lidar":
            self.Lidar()
        
        elif self.args.type[0:5] == "Radar":
            self.Radar()

        elif self.args.type == "Camera":
            self.Camera()
        if self.args.video:
            self.saveVideo()
            
    def checkdir(self): # Check dataImage dir exist 
        if not os.path.isdir(vid_path):
            os.mkdir(vid_path)
        if not os.path.isdir(saveDir):
            os.mkdir(saveDir)
        else:
            for files in os.listdir(saveDir):
                os.remove(saveDir+files)
