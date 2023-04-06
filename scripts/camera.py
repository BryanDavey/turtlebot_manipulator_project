#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import tf
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import copy

window_detection_name = 'Object Detection'
cv.namedWindow(window_detection_name)
cv.namedWindow('camera')


class CallbackHandler():
    def __init__(self):
        fx = 2714.286/4
        fy = fx
        cx = 640/8
        cy = 480/8
        self.cameraIntrinsics = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
        self.current_frame = None
        self.new_pic = False
        self.cameraExtrinsics = None
        self.im_2_cam_transform_xz = None
        self.tf_listener = tf.TransformListener()

        # Map stuff
        self.data = None
        self.grid = None
        self.height = None
        self.width = None
        self.map2d = None
        self.newMap = False
        self.map_data = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.mapCallback)
        self.map_pub = rospy.Publisher('/camera_map', OccupancyGrid, queue_size=10)

    def imageCallback(self, data):
        # time = data.header.stamp
        time = rospy.get_rostime()
        # Get turtlebot position
        self.tf_listener.waitForTransform('/odom','/camera_rgb_optical_frame',time,rospy.Duration(1))
        (pos,rot) = self.tf_listener.lookupTransform('/odom','/camera_rgb_optical_frame',time)
        transform_mx = quaternion_matrix(rot)
        for i,dim in enumerate(pos):
            transform_mx[i,3] = dim
        self.cameraExtrinsics = transform_mx
        br = CvBridge()
        full_size = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.cx = int(full_size.shape[0]/8)
        self.cy = int(full_size.shape[1]/8)
        # resize image
        self.current_frame = cv.resize(full_size, (int(640/4),int(480/4)), interpolation = cv.INTER_AREA)
        self.new_pic = True

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

def processImage(handler):
    # hsv_frame = cv.cvtColor(current_frame,cv.COLOR_BGR2HSV)
    frame = handler.current_frame
    frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    (B, G, R) = cv.split(frame)
    frame_threshold = cv.inRange(frame_HSV, (164, 84, 0), (180, 255, 255)).astype(np.uint8)
    thr_B = cv.bitwise_and(B,frame_threshold)
    thr_G = cv.bitwise_and(G,frame_threshold)
    thr_R = cv.bitwise_and(R,frame_threshold)
    merged_thresh = cv.merge([thr_B, thr_G, thr_R])

    kernel = np.array([[0,0,0,0,0],
                       [0,0,0,0,0],
                       [0,1,1,1,0],
                       [0,0,0,0,0],
                       [0,0,0,0,0]]).astype(np.uint8)
    eroded= cv.erode(frame_threshold,kernel,iterations = 1)
    opened = cv.dilate(eroded,kernel,iterations = 1)

    # Pre-bake inverse for speed
    invIntrinsics = np.linalg.inv(handler.cameraIntrinsics)
    extrinsics = np.linalg.inv(handler.cameraExtrinsics)
    RangeDataXY_px = np.empty((0,2),int)
    for u in range(frame_threshold.shape[0]):
        for v in range(frame_threshold.shape[1]):
            if (frame_threshold[u,v]):
                pt = np.array([u,v,1])
                # Apply pixel to camera coord transform
                pt = np.matmul(handler.im_2_cam_transform_xz,pt)
                # Normalize for scaling factor
                pt = pt/pt[2]
                point_wrt_origin_m = np.matmul(extrinsics,(pt[0],0.1,pt[1],1))
                point_wrt_origin_px = (np.rint(world2map(handler,point_wrt_origin_m[0],point_wrt_origin_m[1]))).astype(int)
                pt_wrt_camera = world2map(handler,pt[0]+1,pt[1])
                RangeDataXY_px = np.vstack([RangeDataXY_px,pt_wrt_camera])
    for i in RangeDataXY_px:
        if (i[0] < 0 or i[0] >= handler.map_data.shape[0] or i[1] < 0 or i[1] >= handler.map_data.shape[1]):
            continue
        handler.map_data[i[0],i[1]] = 100
    path_map = copy.deepcopy(handler.grid)
    path_map.data = tuple(handler.map_data.flatten())
    handler.map_pub.publish(path_map)


    cv.imshow(window_detection_name, opened)

    cv.imshow("camera", handler.current_frame)
    # cv.imwrite("red_square.jpg",handler.current_frame)
    cv.waitKey(1)
    

if __name__ == '__main__':
    rospy.init_node('video_sub_py',anonymous=True)
    handler = CallbackHandler()
    rospy.Subscriber('/camera/rgb/image_raw',Image,handler.imageCallback)

    input_pts = np.float32([[317,152], [234, 133], [87,133], [0,151]])
    output_pts = np.float32([[0.482, 0.7982],[-0.5172, 0.8183],[-0.4881, 1.8147],[0.5110, 1.7884]])
    handler.im_2_cam_transform_xz = cv.getPerspectiveTransform(input_pts,output_pts)
    # y is 0.1 always
    for i in handler.im_2_cam_transform_xz:
        pt = i/i[2]
        print(pt)
    
    while(not handler.new_pic or handler.height == None):
        # Wait until first map has been published
        print('waiting')
        rospy.sleep(0.1)
    handler.map_data = np.zeros([handler.width,handler.height],int)
    print('new_pic found')
    #main loop
    while not rospy.is_shutdown():
        break
        if (handler.new_pic):
            handler.new_pic = False
            processImage(handler)

    cv.destroyAllWindows()