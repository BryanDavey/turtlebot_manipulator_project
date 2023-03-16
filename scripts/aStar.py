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

grid_size = (0, 0)
yaw_thresh = 3
turtle_pos = (-1,-1,-1)
turtle_rot = (-1,-1,-1,-1)
resolution = 0.05
astar_node_path = np.array(np.array([]))
goalx = 2.0
goaly = 2.0
map2d = np.array([])
dilated_map=np.array([])

lock = threading.Lock()




# Handles Turtlebot odometery
class Turtlebot:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.dilatedmap_pub = rospy.Publisher('/dilated_map', OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.pos = np.array([])
        self.ort = Quaternion()
        self.start_pos = None

    def odom_callback(self, data):
        self.pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.ort = data.pose.pose.orientation

turtle_bot = Turtlebot()

# # Map grid given in the assignment
# #     -9 -8 -7 -6 -5 -4 -3 -2 -1 | 1  2  3  4  5  6  7  8  9
# #      0  1  2  3  4  5  6  7  8 | 9 10 11 12 13 14 15 16 17
# map = [0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 1, 0, 0, 0,  # 0  10
#        0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 1, 0, 0, 0,  # 1  9
#        0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 2  8
#        1, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 3  7
#        0, 1, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 4  6
#        0, 0, 1, 0, 0, 0, 1, 1, 1,  1, 1, 1, 0, 0, 0, 0, 0, 0,  # 5  5
#        0, 0, 1, 0, 0, 0, 1, 1, 1,  1, 1, 1, 0, 0, 0, 0, 0, 0,  # 6  4
#        0, 0, 0, 1, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 1, 0,  # 7  3
#        0, 0, 0, 0, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 1, 1,  # 8  2
#        0, 0, 0, 0, 1, 1, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 1, 1,  # 9  1
#        #                         0                             # -----
#        0, 0, 0, 0, 0, 1, 0, 0, 0,  0, 0, 0, 0, 0, 0, 1, 1, 1,  # 10 -1
#        0, 0, 0, 0, 0, 1, 1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 1, 0,  # 11 -2
#        0, 0, 0, 0, 0, 0, 1, 1, 1,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 12 -3
#        0, 0, 0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 13 -4
#        0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 14 -5
#        0, 0, 0, 0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0, 0, 0, 0,  # 15 -6
#        0, 0, 0, 0, 0, 0, 0, 0, 1,  1, 0, 0, 0, 1, 1, 1, 1, 0,  # 16 -7
#        0, 0, 0, 0, 0, 0, 0, 0, 1,  1, 1, 0, 0, 1, 1, 1, 1, 0,  # 17 -8
#        0, 0, 0, 0, 0, 0, 0, 1, 1,  1, 0, 0, 0, 1, 1, 1, 1, 0,  # 18 -9
#        0, 0, 0, 0, 0, 0, 0, 0, 1,  1, 0, 0, 0, 1, 1, 1, 1, 1]  # 19 -10

# Converts node indices to bot's xy position
def node_to_world_pos(node: np.ndarray):
    center_x = grid_size[1] // 2  # 9
    center_y = grid_size[0] // 2  # 10
    if node[0] < grid_size[1] // 2:
        x = (node[0] - center_x)
    else:
        x = (node[0] - center_x) + 1

    if node[1] < grid_size[0] // 2:
        y = (center_y - node[1])
    else:
        y = (center_y - node[1]) - 1
    return np.array([x, y])

def map_callback(grid):
    global turtle_bot
    global astar_node_path
    global map2d
    global turtle_world_listener
    global turtle_pos
    global turtle_rot
    global grid_size
    global resolution

    data = grid.data
    height = grid.info.height
    width = grid.info.width
    grid_size = [height,width]
    resolution = grid.info.resolution

    raw_map = np.array(data).reshape((height, width))
    # Compensating for y-axis being inverted
    raw_map = np.flip(raw_map, axis=1)

    # rospy.loginfo("unique values: {}".format(np.unique(map2d)))
    # mask = np.where(raw_map > 0.2)
    # for i in range(len(mask[0])):
    #     raw_map[mask[0][i]][mask[1][i]] = 1
    mask = np.where(raw_map < 0.0)
    for i in range(len(mask[0])):
        raw_map[mask[0][i]][mask[1][i]] = 0

    struct2= np.ones((5,5))
    dilated_map= ndimage.binary_dilation(raw_map,structure=struct2).astype(raw_map.dtype)
    # dilated_map=np.flip(dilated_map, axis=1)
    # dilated_map = np.rot90(dilated_map)

    dilmap = grid
    dilmap.data = tuple(dilated_map.flatten())
    turtle_bot.dilatedmap_pub.publish(dilmap)

    try:
        lock.acquire()
        map2d=dilated_map
        rospy.logerr("callback acquired lock.")
        printstr = "unique values: {}".format(np.unique(map2d))
        rospy.logerr(printstr)
    except:
        rospy.logerr("callback failed to acquire lock.")
    finally:
        lock.release()


# Shifts the bots postion to center of the grid tile - improves visually
def center_offset(vec):
    if vec[0] < 0:
        offset_x = 0.5
    else:
        offset_x = -0.5
    if vec[1] < 0:
        offset_y = 0.5
    else:
        offset_y = -0.5
    return np.array([vec[0] + offset_x, vec[1] + offset_y])

# Converts bot's xy position to node indices
def world_pos_to_node(pos: np.ndarray):
    center_x = grid_size[1] // 2  # 9
    center_y = grid_size[0] // 2  # 10
    if pos[0] > 0:
        i = center_x + pos[0] - 1
    else:
        i = center_x + pos[0]

    if pos[1] > 0:
        j = center_y - pos[1]
    else:
        j = center_y - pos[1] - 1
    return np.array([i, j])


# Defines A-star node
class Node:
    # pos is the position of node indices
    pos: np.ndarray = np.array([])
    gcost: int = 0
    hcost: int = 0
    fcost: int = 0
    parent = None

    # To remove same node when calling list.remove function
    def __eq__(self, other):
        if not isinstance(other, Node):
            return NotImplemented
        return self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1]  # refine as needed


# A-star algorithm implementation
def astar_pathfinder(bot_pos_in_node, goal_pos_in_node):
    try:
        lock.acquire()
        astar_map = map2d
        rospy.logerr("pathfinder acquired lock.")
        printstr = "unique values: {}".format(np.unique(astar_map))
        rospy.logerr(printstr)
    except:
        rospy.logerr("pathfinder failed to acquire lock.")
    finally:
        lock.release()
    astar_map = np.rot90(astar_map)
    e = 5
    open_nodes: list = []
    closed_nodes: list = []
    rospy.loginfo(f"bot_node: {bot_pos_in_node}, goal_node: {goal_pos_in_node}")
    bot_node = Node()
    bot_node.pos = bot_pos_in_node
    bot_node.pos[0] = int(round(bot_node.pos[0]))
    bot_node.pos[1] = int(round(bot_node.pos[1]))
    open_nodes.append(bot_node)
    while open_nodes:
        current_node: Node = Node()
        min_fcost = np.inf
        for node in open_nodes:
            gcost = np.linalg.norm(node_to_world_pos(node.pos) - node_to_world_pos(bot_pos_in_node))
            hcost = np.linalg.norm(node_to_world_pos(goal_pos_in_node) - node_to_world_pos(node.pos))
            fcost = gcost + e * hcost
            if (fcost < min_fcost):
                min_fcost = fcost
                current_node = node
                current_node.gcost = gcost
                current_node.hcost = hcost
                current_node.fcost = fcost
        open_nodes.remove(current_node)
        closed_nodes.append(current_node)

        curr2goal_dist = np.linalg.norm(node_to_world_pos(goal_pos_in_node) - node_to_world_pos(current_node.pos))
        # rospy.loginfo(f"Working: {curr2goal_dist}")
        node_path = []
        if curr2goal_dist < 0.5:
            search_node = current_node
            while True:
                if search_node.parent is None:
                    break
                node_path.append(search_node.pos)
                search_node = search_node.parent
            break
            # return np.flip(np.array(node_path), axis=0)

        st_path_blocked = [False, False, False, False]

        # Checks if there are no neighbouring nodes which is not accessible
        def check_straight_elements(m, n):
            if m == -1 and n == 1:
                return st_path_blocked[0] and st_path_blocked[1]
            if m == 1 and n == 1:
                return st_path_blocked[1] and st_path_blocked[2]
            if m == 1 and n == -1:
                return st_path_blocked[2] and st_path_blocked[3]
            if m == -1 and n == -1:
                return st_path_blocked[3] and st_path_blocked[0]
            return False

        for index, (nx, ny) in enumerate([(-1, 0), (0, 1), (1, 0), (0, -1), (-1, 1), (1, 1), (1, -1), (-1, -1)]):
            neighbor_node = Node()
            neighbor_node.pos = np.array([current_node.pos[0] + nx, current_node.pos[1] + ny])
            if neighbor_node.pos[0] < 0 or neighbor_node.pos[1] < 0:
                continue
            if neighbor_node.pos[0] >= grid_size[1] or neighbor_node.pos[1] >= grid_size[0]:
                continue
            if neighbor_node in closed_nodes:
                continue
            if astar_map[int(round(neighbor_node.pos[0])), int(round((grid_size[0] - 1) - neighbor_node.pos[1]))] == 1:
                if index < 4:
                    st_path_blocked[index] = True
                continue
            # checks if vertical or horizontal path is blocked
            if index >= 4 and check_straight_elements(nx, ny):
                continue

            neighbor_node.gcost = current_node.gcost + np.linalg.norm(node_to_world_pos(neighbor_node.pos) - node_to_world_pos(current_node.pos))
            neighbor_node.hcost = np.linalg.norm(node_to_world_pos(goal_pos_in_node) - node_to_world_pos(neighbor_node.pos))
            neighbor_node.fcost = neighbor_node.gcost + e * neighbor_node.hcost
            neighbor_node.parent = current_node

            for node in open_nodes:
                if neighbor_node == node:
                    if neighbor_node.gcost > node.gcost:
                        continue

            if neighbor_node not in open_nodes:
                open_nodes.append(neighbor_node)
    rviz_path = Path()
    if len(node_path) > 1:
        for node in node_path:
            this_pose = PoseStamped()
            this_pose.pose.position.x = node[0]
            this_pose.pose.position.y = node[1]
            this_pose.pose.position.z = 0
            # this_pose.header.frame_id = 
            rviz_path.poses.append(this_pose)
        turtle_bot.path_pub.publish(rviz_path)
        return np.flip(np.array(node_path), axis=0)
    else:
        return np.array([0,0,0,0])

def plot_path_to_gui(astar_node_path,plot_map,turtle_pos,goal_pos):
    global resolution
    mask = np.where(plot_map < 0.0)
    for i in range(len(mask[0])):
        plot_map[mask[0][i]][mask[1][i]] = 0
    
    # Compensating for y-axis being inverted
    # map2d = np.flip(map2d, axis=1)

    astar_world_path = []
    print_astar_node_path = []
    for node in astar_node_path:
        print_astar_node_path.append(tuple(node))
        # astar_world_path.append(tuple(center_offset(node_to_world_pos(node))))
        astar_world_path.append(center_offset(node_to_world_pos(node)))
    astar_world_path = np.array(astar_world_path)
    # rospy.loginfo(f"\nA-star Node Path:\n {print_astar_node_path}")
    # rospy.loginfo(f"\nA-star World Path:\n {astar_world_path}")
    # map_for_print = map2d
    # for i, j in astar_node_path:
    #     i = int(round(i))
    #     j = int(round(j))
    #     map_for_print[i, (grid_size[0] - 1) - j] = 5
    # map_for_print = np.flip(map2d.T, axis=0)
    # string_map = str(map_for_print).replace(' [', '').replace('[', '').replace(']', '').replace('0', ' ').replace('5', '*')
    # rospy.loginfo(f"\nMap:\n{string_map}")
    
    turtle_node_pos = world_pos_to_node([turtle_pos[0]/resolution,turtle_pos[1]/resolution])
    goal_node_pos = world_pos_to_node([goalx/resolution,goaly/resolution])
    # Path Visualization
    plt.cla()
    # plt.plot(np.where(map2d == 0)[0]-200, np.where(map2d == 0)[1]-200, marker='.', color='blue', linestyle="")
    plt.plot(np.where(plot_map == 1)[0], np.where(plot_map == 1)[1], marker='s', color='black', linestyle="")
    plt.plot(astar_node_path[:, 0], astar_node_path[:, 1], marker='o', color='red')
    # plt.plot(astar_world_path[:, 0], astar_world_path[:, 1], marker='o', color='red')
    # plt.plot(goalx/resolution,goaly/resolution,marker='o',color='orange')
    plt.plot(goal_node_pos[0],goal_node_pos[1],marker='o',color='orange')
    # plt.plot(turtle_pos[0]/resolution,turtle_pos[1]/resolution,marker='o',color='green')
    plt.plot(turtle_node_pos[0],turtle_node_pos[1],marker='o',color='green')
    plt.title("Plotting turtlebot")
    plt.show(block=False)
    plt.pause(0.1)
    #plt.close("all")
    rospy.loginfo("Initiating Path Execution.")

def init(dilated_map_pub):
    # global new_path
    global astar_node_path
    global turtle_world_listener
    global turtle_pos
    global turtle_rot
    rate = rospy.Rate(10)
    current_node = 0
    world_path = list()

    current_goal_dist = 0.05/resolution

    goal_pos = world_pos_to_node(np.array([round(goalx/resolution + 0.01), round(goaly/resolution)]))

    

    # Main Loop
    while not rospy.is_shutdown():
        world_path.clear()
        try:
            (turtle_pos,turtle_rot) = turtle_world_listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
            printstr = "turtle_pos: [{},{}]".format(turtle_pos[0],turtle_pos[1])
            rospy.loginfo(printstr)
            # start_pos = world_pos_to_node(np.array([first_odom.pose.pose.position.x, first_odom.pose.pose.position.y]))
            start_pos = world_pos_to_node(np.array([turtle_pos[0]/resolution, turtle_pos[1]/resolution]))
            astar_node_path = astar_pathfinder(start_pos, goal_pos)
            astar_node_path = np.vstack((start_pos, astar_node_path))
            for node in astar_node_path:
                # world_path.append(tuple(center_offset(node_to_world_pos(node))))
                world_path.append(tuple(node_to_world_pos(node)))
            plot_path_to_gui(astar_node_path,map2d,turtle_pos,goal_pos)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Error fetching /base_footprint /map tf")
            continue
        except ValueError:
            rospy.logerr("astar_node_path is empty!")
            continue
        # if turtle_bot.pos.shape[0] <= 0:
        #     continue
        
        turtle_pos = [turtle_pos[0]/resolution,turtle_pos[1]/resolution]
       
        # Handles bot turning and movement according to goal node
        try:
            local_goal = np.array(world_path[current_node])
        except IndexError:
            rospy.logerr("IndexError, current_node counter too large. Exiting.")
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            turtle_bot.cmd_pub.publish(twist_msg)
            rate.sleep()
            break
        printstr = "turtle_bot.pos: {},turtle_pos: {}".format(turtle_bot.pos,[turtle_pos[0], turtle_pos[1]])
        rospy.loginfo(printstr)
        distance = np.linalg.norm(turtle_pos - local_goal)
        # distance = np.linalg.norm(turtle_bot.pos - local_goal)
        goal_yaw = np.arctan2(turtle_pos[1] - local_goal[1], turtle_pos[0] - local_goal[0])+np.pi
        # goal_yaw = np.arctan2(local_goal[1] - turtle_pos[1], local_goal[0] - turtle_pos[0])+np.pi
        bot_yaw = tf.transformations.euler_from_quaternion((turtle_bot.ort.x,
                                                            turtle_bot.ort.y,
                                                            turtle_bot.ort.z,
                                                            turtle_bot.ort.w))[2]
        rel_yaw = np.arctan2(np.sin(goal_yaw - bot_yaw), np.cos(goal_yaw - bot_yaw))
        # rel_yaw = rel_yaw + np.pi
        # if rel_yaw > np.pi:
        #     rel_yaw = np.abs(np.pi - rel_yaw) - np.pi

        if distance < current_goal_dist and current_node == len(world_path) - 1:
            rospy.loginfo("Path Execution complete")
            twist_msg = Twist()
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            turtle_bot.cmd_pub.publish(twist_msg)
            rate.sleep()
            break
        while distance < current_goal_dist and current_node < len(world_path) - 2:
            local_goal = np.array(world_path[current_node])
            distance = np.linalg.norm(turtle_pos - local_goal)
            current_node += 1
        twist_msg = Twist()
        yaw_kp = np.min([np.abs(rel_yaw), 2]) / 2
        # if (current_node == len(world_path) - 1):
        dist_kp = np.min([distance, 0.5]) / 0.5
        # else:
        #     dist_kp = 1
        # printstr = "world_path: {}".format(world_path)
        # rospy.logerr(printstr)
        # printstr = "rel_yaw: {}".format(np.rad2deg(rel_yaw))
        # rospy.logerr(printstr)
        # printstr = "goal_yaw: {}".format(np.rad2deg(goal_yaw))
        # rospy.logerr(printstr)
        # printstr = "bot_yaw: {}".format(np.rad2deg(bot_yaw))
        # rospy.logerr(printstr)
        # printstr = "local_goal: {}".format(local_goal)
        # rospy.logerr(printstr)
        # printstr = "turtle_pos: {}".format(turtle_pos)
        # rospy.logerr(printstr)
        if rel_yaw < -np.deg2rad(yaw_thresh) and distance > current_goal_dist:
            twist_msg.angular.z = -3 * yaw_kp

        elif rel_yaw > np.deg2rad(yaw_thresh) and distance > current_goal_dist:
            twist_msg.angular.z = 3 * yaw_kp

        if distance > current_goal_dist and rel_yaw < np.deg2rad(yaw_thresh)*3:
            printstr = "dist_kp: {}, linear.x: {}".format(dist_kp,0.1*dist_kp)
            rospy.logerr(printstr)
            twist_msg.linear.x = 0.2 * dist_kp
        else:
            twist_msg.linear.x = 0.0
        turtle_bot.cmd_pub.publish(twist_msg)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('astar', anonymous=True)
    turtle_world_listener = tf.TransformListener()
    
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
    dilated_map_pub = rospy.Publisher('/dilated_map', OccupancyGrid, queue_size=10)

    first_odom = rospy.wait_for_message("/odom", Odometry)
    first_map = rospy.wait_for_message("/map",OccupancyGrid)
    map2d = np.array(first_map.data).reshape((first_map.info.height, first_map.info.width)).T
    

    # Start main loop
    init(dilated_map_pub)
