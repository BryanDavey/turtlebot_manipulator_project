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

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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

    def mapCallback(self, grid):
        self.grid = grid
        self.data = grid.data
        self.height = grid.info.height
        self.width = grid.info.width
        self.map2d = np.array(self.data).reshape((self.height, self.width))
        self.newMap = True
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
    
    now = rospy.get_rostime()
    

def laserCallback(data):
    incrementAngle = data.angle_increment
    maxAngle = data.angle_max
    minAngle = data.angle_min
    rangeData = data.ranges

    
    (lidar_pos,Lidar_rot_quart) = tf_listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
    Lidar_rot = R.from_quat([Lidar_rot_quart])
    Lidar_rot = Lidar_rot.as_matrix()
    Lidar_Tform=[[ Lidar_rot[0,0,0], Lidar_rot[0,0,1], Lidar_rot[0,0,2], lidar_pos[0]],
                 [ Lidar_rot[0,1,0], Lidar_rot[0,1,1], Lidar_rot[0,1,2], lidar_pos[1]],
                 [ Lidar_rot[0,2,0], Lidar_rot[0,2,1], Lidar_rot[0,2,2], lidar_pos[2]],
                 [0, 0, 0, 1]]

    # rospy.loginfo("Calculated number of lasers: %i, len() %i:",numberOfLasers,len(rangeData))
    # rospy.loginfo("First element: %f, Last: %f", rangeData[0], rangeData[len(rangeData)-1])
    # rospy.loginfo(rangeData)
    # rospy.loginfo(len(rangeData))
    RangeDataXY = np.array(([[0.00,0.00,0.00,1.00]]*360),dtype=float)
    Lidar_points_to_origin_m=np.array(([[0.00,0.00,0.00,1.00]]*360),dtype=float)
    for i in range(len(rangeData)):
        if rangeData[i] <= 4:
            rad= np.deg2rad(i)
            x=round(np.cos(rad)*rangeData[i],2)
            x=round(x/0.05)*0.05
            y=round(np.sin(rad)*rangeData[i],2)
            y=round(y/0.05)*0.05

            # RangeDataXYpix=world2map(mapHandler,round(x/0.05)*0.05,round(y/0.05)*0.05)
            RangeDataXY[i]=(int(x),int(y),0,1)
            Lidar_points_to_origin_m[i]=np.matmul(RangeDataXY[i], Lidar_Tform)  # in rads and metter( i think)
            
        else:
            RangeDataXY[i]=(0,0,0,1)
    
    print(Lidar_points_to_origin_m[0])
    # for i in range(len(rangeData)):
    #     Lidar_points_to_origin_pix=world2map(mapHandler,,round(y/0.05)*0.05)
       
    



def talker(mapHandler):
    mapHandlerf =mapHandler
    sub = rospy.Subscriber('/odom', Odometry, odomCallback)
    laserSub = rospy.Subscriber('/scan', LaserScan, laserCallback)

    # Publish enitial command
    twist = Twist()
    twist.linear.x = 0.1

    pub.publish(twist)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        stored_exception=sys.exc_info()

    # rospy.init_node('talker',anonymous=True)
    # rate = rospy.Rate(10) #Hz
    # while not rospy.is_shutdown():
        

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
    
    


    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)