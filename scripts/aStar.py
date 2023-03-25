#!/usr/bin/env python3

import rospy
import threading
import pyastar2d
import copy
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import tf
import time
from scipy import ndimage

import numpy as np
import matplotlib.pyplot as plt

class CallbackHandler():
    def __init__(self):
        # Map stuff
        self.data = None
        self.grid = None
        self.height = None
        self.width = None
        self.map2d = None
        self.newMap = False
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCallback)
        self.map_pub = rospy.Publisher('/aStar_map', OccupancyGrid, queue_size=10)
        self.dilmap_pub = rospy.Publisher('/dilated_map', OccupancyGrid, queue_size=10)
        # Odometry stuff
        self.pos = np.array([])
        self.ort = Quaternion()
        self.turnRadius = 220 #mm
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def mapCallback(self, grid):
        self.grid = grid
        self.data = grid.data
        self.height = grid.info.height
        self.width = grid.info.width
        self.map2d = np.array(self.data).reshape((self.height, self.width))
        self.newMap = True

    def odom_callback(self,data):
        self.pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.ort = data.pose.pose.orientation

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
        self.diagonal = False

    def __eq__(self, other):
        return self.position == other.position

def world2map(mapObject,worldx,worldy):
    res=mapObject.grid.info.resolution# 0.05m/p
    Xoffset= mapObject.grid.info.origin.position.x #offset in m 
    Yoffset = mapObject.grid.info.origin.position.y #offset in m
    # x and y are inverted between map and world
    pixy=int(round((-Xoffset+worldx)*(1/res)))
    pixx=int(round((-Yoffset+worldy)*(1/res)))
    return(pixx,pixy)


def map2world(mapObject,mapx,mapy):
    res = mapObject.grid.info.resolution #0.05m/px
    xOffset = mapObject.grid.info.origin.position.x #-10
    yOffset = mapObject.grid.info.origin.position.y #-10
    # x and y are inverted between map and world
    worldy = mapx*res + xOffset
    worldx = mapy*res + yOffset
    return(worldx,worldy)

def main(mapHandler):
    #Row then column 
    # (0,1) is 0th row 1st column
    # Diameter 22
           #0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
    map22 = [[0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0],#0
             [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],#1
             [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],#2
             [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],#3
             [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],#4
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#5
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#6
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#7
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#8
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#9
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#10
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#1
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#2
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#3
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#4
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#5
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#6
             [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],#7
             [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],#8
             [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],#9
             [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],#20
             [0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0]]#1

    # Diameter 16
    #         1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6
    map16 = [[0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0],#1
             [0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0],#2
             [0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0],#3
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#4
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#5
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#6
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#7
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#8
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#9
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#0
             [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],#1
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#2
             [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],#3
             [0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0],#4
             [0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0],#5
             [0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0]]#6
    radius_mask=np.array([np.array(xi) for xi in map16])

    tf_listener = tf.TransformListener()


    # path = astar(maze, start, end)
    # print(path)

    while(not mapHandler.newMap):
        # Wait until first map has been published
        rospy.sleep(0.1)
    while not rospy.is_shutdown():
        # Read new map and publish dilated map
        if (mapHandler.newMap):
            mapHandler.newMap = False
            map2d = np.copy(mapHandler.map2d)
            mask = np.where(map2d < 0.0)
            for i in range(len(mask[0])):
                map2d[mask[0][i]][mask[1][i]] = 0
            map2d = np.divide(map2d,100).astype(int)
            dilated_map = ndimage.binary_dilation(map2d,structure=radius_mask).astype(map2d.dtype).astype(np.float32)
            dilated_map[dilated_map == 1] = np.inf
            dilated_map[dilated_map == 0] = 1
            assert dilated_map.min() == 1, "cost of moving must be at least 1"
            dilation_time = rospy.get_rostime()

            # Publish dilated map
            dilated_2pub = np.copy(dilated_map)
            dilated_2pub[dilated_map > 1] = 100
            dilated_2pub[dilated_map == 1] = 0
            dilmap = copy.deepcopy(mapHandler.grid)
            dilmap.data = tuple(dilated_2pub.astype(int).flatten())
            mapHandler.dilmap_pub.publish(dilmap)

        # Get turtlebot position
        (turtle_pos,turtle_rot) = tf_listener.lookupTransform('/map','/base_footprint',rospy.Time(0))

        rospy.loginfo('bot position (x,y)m: ({},{})'.format(turtle_pos[0],turtle_pos[1]))
        start = world2map(mapHandler,turtle_pos[0], turtle_pos[1])
        rospy.loginfo('bot pos in map coords: {}'.format(start))
        end = world2map(mapHandler,-1.5,-1.5)

        # Compute A* path
        path = pyastar2d.astar_path(dilated_map, np.array(start), np.array(end), allow_diagonal=True)
        astar_time = rospy.get_rostime() - dilation_time
        path_data = np.zeros([mapHandler.width,mapHandler.height])
        if path is not None:
            if len(path) > 1:
                for i in path:
                    path_data[i[0],i[1]] = 100
                # Publish map 
                path_data = np.asarray(path_data, dtype = 'int')
                path_map = copy.deepcopy(mapHandler.grid)
                path_data = list(path_data.ravel())
                path_map.data = path_data
                mapHandler.map_pub.publish(path_map)
            else:
                rospy.logerr('path not long enough?')
        else:
            rospy.logerr('Invalid path.')

        # Run at max 20Hz
        rospy.sleep(0.05)

def testCallback(data):
    rospy.logerr('data received')

if __name__ == '__main__':

    rospy.init_node('aStar', anonymous=True)

    mapHandler = CallbackHandler()
    main(mapHandler)
    rospy.spin()


    