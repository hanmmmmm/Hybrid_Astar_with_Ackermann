#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid 
from std_msgs.msg import Float32MultiArray
import cv2
import yaml
from time import time 
from random import randint 


def loadImage():

    # file_dir = os.path.dirname(os.path.realpath('__file__'))
    # file_dir = "/home/jf/.ros"
    file_dir = "/home/jf/Hybrid_Astar_with_Ackermann/src/hawa_pathplan/src/test_env_tools/maps"
    print (file_dir)


    # map_image_file = file_dir + "/mymap.pgm"
    # map_image_file = file_dir + "/map_files/parking_1.pgm"
    # map_image_file = file_dir + "/map_files/office_1.pgm"
    map_image_file = file_dir + "/map.png"
    # map_image_file = file_dir + "/test_square.png"


    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

    print(f"map_image_file  {map_image_file}")

    img = cv2.imread(map_image_file, cv2.IMREAD_UNCHANGED)

    # # stright block
    # img[20:80, 90:100 ] = [0,0,0]

    # # square trap
    # img[10:90, 30:80 ] = [0,0,0]
    # img[30:80, 40:70 ] = [254,254,254]

    # # # cover robot in obstble
    # img[50:70, 50:60 ] = [0,0,0]

    # print(img)

    print(img.shape)

    # with open(file_dir + "/map_files/mymap.yaml", 'r') as file:
    with open(file_dir + "/mymap.yaml", 'r') as file:
        map_yaml = yaml.safe_load(file)

    print(map_yaml)

    # pxset = set()
    # for y in range(img.shape[0]):
    #     for x in range(img.shape[1]):
    #         px = img[y][x]
    #         # pxset.add(px)
    #         print(px)

    # print(f"pxset {pxset}.")

    pxset = set()
    img_1d_list = []
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            px = img[img.shape[0]-1-y][x][3]
            # px = int(100.0 - float(px)*100.0/255.0)
            px = int(float(px)*100.0/255.0)
            px = max( px, 0.0 )
            img_1d_list.append( px )
            pxset.add(px)
    print(f"pxset {pxset}.")


    img_1d_list_backup = img_1d_list.copy()

    return img_1d_list_backup,img,map_yaml



# gridmap_msg = OccupancyGrid()

# gridmap_msg.data = img_1d_list
# gridmap_msg.header.frame_id = "map"
# gridmap_msg.header.seq = 0

# gridmap_msg.info.height = img.shape[0]
# gridmap_msg.info.width = img.shape[1]
# gridmap_msg.info.resolution = map_yaml['resolution']
# gridmap_msg.info.origin.position.x = map_yaml['origin'][0]
# gridmap_msg.info.origin.position.y = map_yaml['origin'][1]


# OBS_CENTER_X = int(img.shape[0]/2)
# OBS_CENTER_Y = int(img.shape[1]/2)

# OBS_WIDTH_GRID = 8
# OBS_HEIHT_GRID = 8


# def obs_cb(msg:Float32MultiArray):
#     global OBS_CENTER_X, OBS_CENTER_Y, OBS_WIDTH_GRID, OBS_HEIHT_GRID
#     OBS_CENTER_X += msg.data[1] * 0.2
#     OBS_CENTER_Y -= msg.data[0] * 0.2
#     OBS_WIDTH_GRID += msg.data[3] * 0.2
#     OBS_HEIHT_GRID += msg.data[2] * 0.2
#     OBS_HEIHT_GRID = max(1, OBS_HEIHT_GRID)
#     OBS_WIDTH_GRID = max(1, OBS_WIDTH_GRID)


# rospy.init_node("test_gridmap_puber")

# map_puber = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

# obs_move_suber = rospy.Subscriber("/obstacle_motion", Float32MultiArray, obs_cb, queue_size=1)

# TOTAL_WIDTH = img.shape[0]
# TOTAL_HEIHT = img.shape[1]


# rate = rospy.Rate(10)

# last_obs_time = time()

# while not rospy.is_shutdown():

    

#     img_1d_list = img_1d_list_backup.copy()

#     _x_start = max(int(OBS_CENTER_X-OBS_WIDTH_GRID/2), 0)
#     _x_end = min(int(OBS_CENTER_X+OBS_WIDTH_GRID/2), TOTAL_WIDTH)

#     _y_start = max(int(OBS_CENTER_Y-OBS_HEIHT_GRID/2), 0)
#     _y_end = min(int(OBS_CENTER_Y+OBS_HEIHT_GRID/2), TOTAL_HEIHT)

#     for xs in range(_x_start, _x_end):
#         for ys in range(_y_start, _y_end):
#             img_1d_list[int(ys)*TOTAL_WIDTH+int(xs)] = 100

#     # img_1d_list[int(OBS_CENTER_Y)*TOTAL_WIDTH+int(OBS_CENTER_X)] = 100

#     gridmap_msg.data = img_1d_list

#     map_puber.publish(gridmap_msg)
#     rate.sleep()





class ClassMapPublisher(Node):
    def __init__(self):
        super().__init__('node_test_map_pub')
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 1)

        self.img_1d_list, self.img, self.map_yaml = loadImage()
        
        timer_period = 0.1  #sec
        self.timer = self.create_timer(timer_period, self.publish_all)

    def publish_all(self):
        
        gridmap_msg = OccupancyGrid()

        gridmap_msg.data = self.img_1d_list
        gridmap_msg.header.frame_id = "map"
        # gridmap_msg.header.seq = 0

        gridmap_msg.info.height = self.img.shape[0]
        gridmap_msg.info.width = self.img.shape[1]
        gridmap_msg.info.resolution = self.map_yaml['resolution']
        gridmap_msg.info.origin.position.x = self.map_yaml['origin'][0]
        gridmap_msg.info.origin.position.y = self.map_yaml['origin'][1]

        self.map_publisher.publish(gridmap_msg) 


if __name__ == "__main__":

    rclpy.init()

    publisher = ClassMapPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
