#!/usr/bin/env python3
import numpy as np
import time, sys

import rospy
import tf
from std_msgs.msg import String
import geometry_msgs as geo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations as ts

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

wall_detected = False
scanRange = 30

def odomCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "Message: (%f,%f,%f)", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    now = rospy.get_rostime()
    rospy.logwarn("current time: %i %i", now.secs, now.nsecs)

def laserCallback(data):
    global wall_detected
    global scanRange
    incrementAngle = data.angle_increment
    maxAngle = data.angle_max
    minAngle = data.angle_min
    numberOfLasers = maxAngle/incrementAngle
    rangeData = data.ranges
    rospy.loginfo("Calculated number of lasers: %i, len() %i:",numberOfLasers,len(rangeData))
    rospy.loginfo("First element: %f, Last: %f", rangeData[0], rangeData[len(rangeData)-1])

    # ts.euler_from_quaternion()

    if wall_detected == False:
        scanRange = 30
    if wall_detected == True:
        scanRange = 60

    ranges = [0] * scanRange

    wall_detected = False
    for i in range(0, scanRange):
            index = int(i-scanRange/2.0)
            if index < 0:
                index += 360
            ranges[i] = rangeData[index]
            if rangeData[index] < 0.5:
                wall_detected = True

    rospy.loginfo("Ranges len: %i",len(ranges))
    twist = Twist()
    if wall_detected == True:
        twist.linear.x = 0
        twist.angular.z = 0.5
    else:
        twist.linear.x = 0.2
        twist.angular.z = 0
    
    pub.publish(twist)



def talker():
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
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("This is sent after talker() is called.")

    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)