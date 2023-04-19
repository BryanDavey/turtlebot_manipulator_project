#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as R
import numpy as np
import time, sys
import math
import rospy
import tf
from std_msgs.msg import String
import geometry_msgs as geo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf import transformations as ts
import copy


# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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
        self.map_pub = rospy.Publisher('/aStar_map', OccupancyGrid, queue_size=2)
        self.lidar_map_pub = rospy.Publisher('/lidar_map', OccupancyGrid, queue_size=2)
        self.dilmap_pub = rospy.Publisher('/dilated_map', OccupancyGrid, queue_size=10)
        # Odometry stuff

        #Lidar stuff
        self.Map_data = None
        self.incrementAngle = None
        self.rangeData = None
        self.lidar_pos = None
        self.Lidar_rot_quart = None
        self.newLaserScan = False

    def mapCallback(self, grid):
        self.grid = grid
        self.data = grid.data
        self.height = grid.info.height
        self.width = grid.info.width
        self.map2d = np.array(self.data).reshape((self.height, self.width))
        self.newMap = True

    def laserCallback(self, data):
        # print(data.header.stamp)
        tf_listener.waitForTransform('/map','/base_scan',data.header.stamp,rospy.Duration(1))
        (self.lidar_pos,self.Lidar_rot_quart) = tf_listener.lookupTransform('/map','/base_scan',data.header.stamp)
        self.incrementAngle = data.angle_increment
        self.rangeData = data.ranges
        self.newLaserScan = True


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
def odomCallback(data):
    
    now =data.header.stamp
    

def laserCallback(data):
    now = data.header.stamp
    print(now)
    print(rospy.get_time())
    print(rospy.get_rostime())
    incrementAngle = data.angle_increment
    maxAngle = data.angle_max
    minAngle = data.angle_min
    rangeData = data.ranges
    tf_listener.waitForTransform('/map','/base_scan',now,rospy.Duration(1))
    (lidar_pos,Lidar_rot_quart) = tf_listener.lookupTransform('/map','/base_scan',now)

       
    
def processNewScan(mapHandler):
    Lidar_rot = R.from_quat([mapHandler.Lidar_rot_quart])
    Lidar_rot = Lidar_rot.as_matrix()
    Lidar_Tform=[[ Lidar_rot[0,0,0], Lidar_rot[0,0,1], Lidar_rot[0,0,2], mapHandler.lidar_pos[0]],
                 [ Lidar_rot[0,1,0], Lidar_rot[0,1,1], Lidar_rot[0,1,2], mapHandler.lidar_pos[1]],
                 [ Lidar_rot[0,2,0], Lidar_rot[0,2,1], Lidar_rot[0,2,2], mapHandler.lidar_pos[2]],
                 [0, 0, 0, 1]]
    RangeDataXY_px = np.empty((0,2),int)
    # mapHandler.Map_data = np.zeros((mapHandler.width,mapHandler.height),int)

    for i,range in enumerate(mapHandler.rangeData):
        if not np.isinf(range):
            rad= np.deg2rad(i)
            x=round(np.cos(rad)*range,2)
            y=round(np.sin(rad)*range,2)

            point_wrt_origin_m = np.matmul(Lidar_Tform,(x,y,0,1))
            point_wrt_origin_px = (np.rint(world2map(mapHandler,point_wrt_origin_m[0],point_wrt_origin_m[1]))).astype(int)
            RangeDataXY_px = np.vstack([RangeDataXY_px,point_wrt_origin_px])
    for i in RangeDataXY_px:
        mapHandler.Map_data[i[0],i[1]] = 100
    path_map = copy.deepcopy(mapHandler.grid)
    path_map.data = tuple(mapHandler.Map_data.flatten())
    mapHandler.lidar_map_pub.publish(path_map)


def talker(mapHandler):
    mapHandlerf =mapHandler
    sub = rospy.Subscriber('/odom', Odometry, odomCallback)
    laserSub = rospy.Subscriber('/scan', LaserScan, mapHandler.laserCallback)

    # # Publish enitial command
    # twist = Twist()
    # twist.linear.x = 0.1

    # pub.publish(twist)

    while(not mapHandler.newMap):
        # Wait until first map has been published
        rospy.sleep(0.1)
    mapHandler.Map_data = np.zeros([mapHandler.width,mapHandler.height],int)
    while(not mapHandler.newLaserScan):
        rospy.sleep(0.1)
    while not rospy.is_shutdown():
        processNewScan(mapHandler)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        stored_exception=sys.exc_info()

    rospy.init_node('talker',anonymous=True)
    rate = rospy.Rate(10) #Hz
        

    #     # if goiung to crash 
    #     #     DONT

if __name__ == '__main__':
    rospy.init_node('turtleDriver')
    rospy.loginfo("This is info from turtleDriver")
    rospy.logwarn("This is a warning from turtleDriver")
    rospy.logerr("This is an error from turtleDriver")
    rospy.logdebug("This is a debug message from turtleDriver")
    mapHandler = CallbackHandler()
    tf_listener = tf.TransformListener()
    try:
        talker(mapHandler)
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("This is sent after talker() is called.")
    
    


    # twist = Twist()
    # twist.linear.x = 0
    # twist.angular.z = 0
    # pub.publish(twist)