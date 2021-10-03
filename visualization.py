
import sys
from process import Process
import argparse

def parse():
    parser = argparse.ArgumentParser(description='Enter which file and type of image to be converted')
    parser.add_argument('--type', type=str, nargs='?', choices=["Lidar3D", "LidarBird", "Camera", "Radar3D", "RadarBird"], required=True, help= '--type LidarBird/Lidar3D/Camera/Radar3D/RadarBird')
    parser.add_argument('--radar', type=int, nargs='*', required=False, help= '--radar 1/0')
    parser.add_argument('--video', default=False, action='store_true')
    parser.add_argument('--frame', type=int, nargs='*', required=True, help= '--frame int int')
    parser.add_argument('--dataset', type=int, nargs='*', required=True, help= '--dataset int')

    # Handle Arguments
    args = parser.parse_args()
    if args.type[0:5] == "Radar" and not args.radar:
        raise NameError("radar flag not set. Please specify --radar 0/1")
    else:
        args.radar = [0] # Set default radar flag
    if args.video:
        args.frame=[0,297]
        print("video flag raised yes, override frame creation from 0 to 297")
    return args

if __name__ == '__main__':

    args = parse() # Return which arguments
    image = Process(args) # Create Class instance of astyx dataset
    image.options()
    

    



