#!/usr/bin/env python3

import rospy
import threading
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
        self.data = None
        self.grid = None
        self.height = None
        self.width = None
        self.map2d = None
        self.newData = False
        self.map_pub = rospy.Publisher('/aStar_map', OccupancyGrid, queue_size=10)

    def mapCallback(self, grid):
        now = rospy.get_rostime()
        rospy.logwarn("current time: %i %i", now.secs, now.nsecs)
        self.grid = grid
        self.data = grid.data
        self.height = grid.info.height
        self.width = grid.info.width
        self.map2d = np.array(self.data).reshape((self.height, self.width))
        self.newData = True
        self.origin = grid.info.origin.position.x

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 100:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

def world2map (mapObject,worldx,worldy):
    res=mapObject.grid.info.resolution# 0.05m/p
    Xoffset= mapObject.grid.info.origin.position.x #off set in m 
    Yoffset = mapObject.grid.info.origin.position.y #offset in m
    pixx=int(round((-Xoffset+worldx)*(1/res)))
    pixy=int(round((-Yoffset+worldy)*(1/res)))
    if (pixx>mapObject.grid.info.width | pixy >mapObject.grid.info.height):
        print("HERE YOU GO BRYAN ARE YOU HAPPY NOW 8===D~ (:)")
    return(pixx,pixy)
def main(mapHandler):
    #Row then column 
    # (0,1) is 0th row 1st column
            #0  1  2  3  4  5  6  7  8  9
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#0
            [1, 1, 0, 0, 1, 0, 0, 0, 0, 0],#1
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#2
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#3
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#4
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],#5
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#6
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#7
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],#8
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]#9


    # path = astar(maze, start, end)
    # print(path)
    while not rospy.is_shutdown():
        print(".")
        if (mapHandler.newData):
            start = world2map(mapHandler,-1, -1)
            end = world2map(mapHandler,1,1)
            print("world2map: (0,0): {}\n".format(world2map(mapHandler,0,0)))
            print(mapHandler.map2d.shape)
            path = astar(mapHandler.map2d, start, end)
            rospy.loginfo('width: %d, height %d',mapHandler.width,mapHandler.height)
            path_data = np.zeros([384,384])
            # path_data = np.zeros(mapHandler.width,mapHandler.height)
            if len(path) > 2:
                for i in path:
                    path_data[i[0],i[1]] = 100
                path_data = np.asarray(path_data, dtype = 'int')
            else:
                rospy.logerr('path not long enough?')
            # res=mapHandler.grid.info.resolution # 0.05m/c
            # center=int(np.floor(10*(1/res)))
            # print(center)
            # path_data[center,center]=100
            path_map = mapHandler.grid
            path_data = list(path_data.ravel())
            path_map.data = path_data
            

            mapHandler.newData = False
            print(mapHandler.newData)
            mapHandler.map_pub.publish(path_map)

        rospy.sleep(1)

def testCallback(data):
    rospy.logerr('data received')

if __name__ == '__main__':

    rospy.init_node('aStar', anonymous=True)

    mapHandler = CallbackHandler()
    map_sub = rospy.Subscriber('/map', OccupancyGrid, mapHandler.mapCallback)
    main(mapHandler)
    rospy.spin()


    