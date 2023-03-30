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
from scipy.ndimage.filters import gaussian_filter

import numpy as np
import matplotlib.pyplot as plt

class Turtlebot:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pos = np.array([])
        self.ort = Quaternion()
        self.start_pos = None

    def odom_callback(self, data):
        self.pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.ort = data.pose.pose.orientation
        # orientation_list = [self.ort.x, self.ort.y, self.ort.z, self.ort.w]
        # (roll, pitch, yaw) =  tf.transformations.euler_from_quaternion(orientation_list)
        # current_yaw = yaw

turtle_bot = Turtlebot()

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
        self.turnRadius = 0#mm
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
    current_node = 6
    rate = rospy.Rate(10)

    #Row then column 
    # (0,1) is 0th row 1st column
  
           #  1 2 3 4 5 6 7 8 9 10 11
    map11= [[0,0,0,1,1,1,1,1,0,0,0], #1
            [0,0,1,1,1,1,1,1,1,0,0], #2
            [0,1,1,1,1,1,1,1,1,1,0], #3
            [1,1,1,1,1,1,1,1,1,1,1], #4
            [1,1,1,1,1,1,1,1,1,1,1], #5
            [1,1,1,1,1,1,1,1,1,1,1], #6
            [1,1,1,1,1,1,1,1,1,1,1], #7
            [1,1,1,1,1,1,1,1,1,1,1], #8
            [0,1,1,1,1,1,1,1,1,1,0], #9
            [0,0,1,1,1,1,1,1,1,0,0], #10
            [0,0,0,1,1,1,1,1,0,0,0]] #11

         #  1 2 3 4 5 6 7
    map7= [[0,0,1,1,1,0,0], #1
           [0,1,1,1,1,1,0], #2
           [1,1,1,1,1,1,1], #3
           [1,1,1,1,1,1,1], #4
           [1,1,1,1,1,1,1], #5
           [0,1,1,1,1,1,0], #6
           [0,0,1,1,1,0,0]] #7
    radius_mask=np.array([np.array(xi) for xi in map7])

    tf_listener = tf.TransformListener()




    while(not mapHandler.newMap):
        # Wait until first map has been published
        rospy.sleep(0.1)

    #main loop
    while not rospy.is_shutdown():
        # Read new map and publish dilated map
        if (mapHandler.newMap):
            mapHandler.newMap = False
            map2d = np.copy(mapHandler.map2d)
            mask = np.where(map2d < 0.0)
            for i in range(len(mask[0])):
                map2d[mask[0][i]][mask[1][i]] = 0
            # map2d = np.divide(map2d,100).astype(int)
            dilated_map = ndimage.binary_dilation(map2d,structure=radius_mask).astype(map2d.dtype).astype(np.float32)
            print('b4 dilated_map.min: {}, max: {}\n'.format(map2d.min(),map2d.max()))
            dilated_map = (gaussian_filter(dilated_map, sigma=2)*1E24).astype(np.float32)
            mask = np.where(dilated_map < 1)
            for i in range(len(mask[0])):
                dilated_map[mask[0][i]][mask[1][i]] = 1
            print('dilated_map.min: {}, max: {}\n'.format(dilated_map.min(),dilated_map.max()))
            # dilated_map[dilated_map == 1] = np.inf
            # dilated_map[dilated_map == 0] = 1
            assert dilated_map.min() == 1, "cost of moving must be at least 1"
            dilation_time = rospy.get_rostime()

            # Publish dilated map
            dilated_2pub = np.copy(dilated_map)
            dilated_2pub = dilated_2pub/dilated_2pub.max()*100
            # dilated_2pub[dilated_map > 1] = 100
            # dilated_2pub[dilated_map == 1] = 0
            dilmap = copy.deepcopy(mapHandler.grid)
            dilmap.data = tuple(dilated_2pub.astype(int).flatten())
            mapHandler.dilmap_pub.publish(dilmap)

        # Get turtlebot position
        (turtle_pos,turtle_rot) = tf_listener.lookupTransform('/odom','/base_footprint',rospy.Time(0))

        # rospy.loginfo('bot position (x,y)m: ({},{})'.format(turtle_pos[0],turtle_pos[1]))
        start = world2map(mapHandler,turtle_pos[0], turtle_pos[1])
        # rospy.loginfo('bot pos in map coords: {}'.format(start))
        # end = world2map(mapHandler,1.3,2.8)
        end = world2map(mapHandler,4,2)

        # Compute A* path
        path = pyastar2d.astar_path(dilated_map, np.array(start), np.array(end), allow_diagonal=True)
    
        path_data = np.zeros([mapHandler.width,mapHandler.height])
        if path is not None:
            if len(path) > 1:
                for i in path:
                    path_data[i[0],i[1]] = 100
                    #rospy.loginfo(f"\npath:\n {path}")
                path_data[0,0] = 100
                path_data[1,0] = 100
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

        
        if (len(path) < 2 or current_node >= len(path)):
            local_goal = np.array(end)
        else:
            local_goal = (path[current_node])  # current goal is the nth node in the astar list
        yaw_thresh = 15
        orientation_list = [turtle_bot.ort.x, turtle_bot.ort.y, turtle_bot.ort.z, turtle_bot.ort.w] # gets the quat of the rotation
        (roll, pitch, yaw) =  tf.transformations.euler_from_quaternion(orientation_list) # gets roll pitch and yah from th quart
        current_yaw = yaw # the yaw of the robot at that moment in time
        distance = np.linalg.norm(start - local_goal) # distance from current node to goal node
        angle_to_goal = np.arctan2(local_goal[0] - start[0], local_goal[1] - start[1]) #- current_yaw
        rel_yaw = np.arctan2(np.sin(angle_to_goal - current_yaw), np.cos(angle_to_goal - current_yaw))
        rel_yaw = rel_yaw + np.pi
        
        turn_cmd = Twist()

        if rel_yaw > np.pi:
             rel_yaw = np.abs(np.pi - rel_yaw) - np.pi

        if (current_yaw < 0):
            current_yaw_0_360 = current_yaw + 2*np.pi
        else:
            current_yaw_0_360 = current_yaw
        if (angle_to_goal < 0):
            angle_to_goal_0_360 = angle_to_goal + 2*np.pi
        else:
            angle_to_goal_0_360 = angle_to_goal
        d = current_yaw_0_360 - angle_to_goal_0_360
        if (d<0):
            d += 2*np.pi

        if d > np.pi:
            turn_cmd.angular.z = (0.05*abs(d)+0.15)
        elif d <= np.pi:
            turn_cmd.angular.z = -(0.05*abs(d)+0.15)
      
        DegreeDiff= np.rad2deg(np.abs((d)))
        if (distance > 0.5) and (DegreeDiff < yaw_thresh):
            turn_cmd.linear.x = 0.02*current_node
        

            
        if distance <1:
                turn_cmd.linear.x = 0.0
                turn_cmd.angular.z = 0.0
                turtle_bot.cmd_pub.publish(turn_cmd)
                rospy.loginfo(f"\n destination reached!")
                break
    
        turtle_bot.cmd_pub.publish(turn_cmd)    


def testCallback(data):
    rospy.logerr('data received')

if __name__ == '__main__':

    rospy.init_node('aStar', anonymous=True)

    mapHandler = CallbackHandler()
    main(mapHandler) 