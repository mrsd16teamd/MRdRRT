#!/usr/bin/python
# -*- encoding: utf-8 -*-

import rospy
import tf
import rospkg

import std_msgs.msg
from std_srvs.srv import Trigger
import geometry_msgs.msg
import nav_msgs.msg
from geometry_msgs.msg import (Twist, TransformStamped, PoseStamped)

import numpy as np
from time import sleep
from prm_planner import PRMPlanner
from mrdrrt_planner import MRdRRTPlanner
import cPickle as pickle
from collections import defaultdict, deque

n_rob = 3
# gconfigs = np.array([[0.2, -0.05], [-0.2, -0.05]]) #hard-coded goal configs, for now
gconfigs = np.array([[0.30, -0.05], [0, 0.30], [-0.30, -0.05]])
class MrdrrtCommanderNode:

    def __init__(self, *args):
        rospack = rospkg.RosPack()
        path = rospack.get_path('mrdrrt')
        map_path = path + '/roadmaps/' + 't_map_prm.p'

        prm = PRMPlanner(n_nodes=300, map_id=1, load=True, visualize=False, filepath=map_path)
        self.mrdrrt = MRdRRTPlanner(prm, n_robots=n_rob, visualize=False)

        self.tf_listener = tf.TransformListener()
        self.map_frame = '/map'
        self.robot_namespaces = ['cozmo'+str(i) for i in range(n_rob)]
        self.robot_frames = [self.robot_namespaces[i] + '/base_link' for i in range(n_rob)]

        self.plan_serv = rospy.Service('mrdrrt_start', Trigger, self.PlanPath)
        self.robots_done_sub = rospy.Subscriber('/goal_reached', std_msgs.msg.Int8, self.GoalReachedCb, queue_size=5)
        self.waypoint_pubs = [rospy.Publisher(self.robot_namespaces[i]+'/goal', PoseStamped, queue_size=3) for i in range(n_rob)]

        print("MRdRRT commander node ready!")

    def GoalReachedCb(self, msg):
        if msg.data == 1:
            self.n_robots_done += 1

    def GetRobotPose(self, robot_id):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frames[robot_id], rospy.Time(0))
            eul = tf.transformations.euler_from_quaternion(rot)
            yaw = eul[2]
            config = np.array([trans[0], trans[1]])
            return config
        except:
            print("Couldn't get transform between " + self.map_frame + " and " + self.robot_frames[robot_id])
            return False

    def AddAnglesToPath(self, point_path):
        """Post-processes path from roadmap before sending to Cozmo.
        Add angles to path
        For angles, just points robot towards next waypoint.
        """
        path_w_angles = []
        for i, point in enumerate(point_path[0:-1]):  # First point -> second to last
            # vec2next = point_path[i+1] - point_path[i]
            # angle = np.arctan2(vec2next[1], vec2next[0])
            # config = np.append(point, angle)
            # path_w_angles.append(config)
            if point[1] > 0: # y>0
                config = np.append(point, -np.pi/2)
            elif point[0] > 0:
                config = np.append(point, np.pi)
            else:
                config = np.append(point, 0)
            path_w_angles.append(config)

        return path_w_angles

    def kahn_topsort(graph):
        in_degree = {u: 0 for u in graph}     # determine in-degree
        for u in graph:                          # of each node
            for v in graph[u]:
                in_degree[v] += 1
        Q = deque()                 # collect nodes with zero in-degree
        for u in in_degree:
            if in_degree[u] == 0:
                Q.appendleft(u)
        L = []     # list for order of nodes
        while Q:
            u = Q.pop()          # choose node of zero in-degree
            L.append(u)          # and 'remove' it from graph
            for v in graph[u]:
                in_degree[v] -= 1
                if in_degree[v] == 0:
                    Q.appendleft(v)
        if len(L) == len(graph):
            return L
        else:                    # if there is a cycle,
            return []            # then return an empty list

    def IsEmptyGraph(graph):
        empty = True
        for key in graph.keys():
            if (len(graph[key]) != 0):
                empty = False
        return empty

    def OrderRobotsOnPath(path):
        orders = []
        n_robots = len(path.keys())
        orders.append([])  # Assume first step is always ok
        for t in range(1, len(path[0])):
            start_pos = [path[r][t-1] for r in range(n_robots)]
            goal_pos = [path[r][t] for r in range(n_robots)]

            # Build dependency graph
            dgraph = defaultdict(list)
            for i in range(n_robots):
                for j in range(n_robots):
                    if i != j:
                        if (goal_pos[i] == start_pos[j]).all():
                            dgraph[j].append(i)
                if len(dgraph[i]) == 0:
                    dgraph[i] = []

            # Order robots
            if (IsEmptyGraph(dgraph)):
                order = []
            else:
                order = kahn_topsort(dgraph)
            orders.append(order)
            print('.')
            print(start_pos)
            print(goal_pos)
            print(dgraph)
            print(order)

        path['o'] = orders
        return path

    def FillPoseMsg(self, point):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x, pose_msg.pose.position.y = point[0], point[1]

        quat = tf.transformations.quaternion_from_euler(0, 0, point[2])
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]

        return pose_msg

    def PlanPath(self, request):
        # Find robots' starting configurations
        # sconfigs = []
        # for i in range(n_rob):
        #     sconfigs.append(self.GetRobotPose(i))
        # sconfigs = np.array(sconfigs)

        # # Get path from mrdrrt - will dictionary of lists of numpy arrays
        # print("Start Configs:\n{}".format(sconfigs))
        # print("Goal Configs:\n{}".format(gconfigs))

        ### Find path
        # path = self.mrdrrt.FindPath(sconfigs, gconfigs)

        ### Load path from pickle
        rospack = rospkg.RosPack()
        path = rospack.get_path('mrdrrt')
        filepath = path + '/paths/decent.p'
        with open(filepath, 'rb') as f:
            path = pickle.load(f)

        n_waypoints = len(path[0])
        n_rob = len(path.keys())

        self.OrderRobotsOnPath(path)
        for r in range(len(path.keys())):
            path[r] =  self.AddAnglesToPath(path[r])
            # print path[r]
        print("num_robots:{}".format(n_rob))
        print("n_waypoints:{}", format(n_waypoints))

        ### Manually constructed path
        # path = {0: [np.array([0.17, -0.05, np.pi]), np.array([0,-0.05,np.pi/2]), np.array([0, 0.17, -np.pi/2]), np.array([0, -0.05, np.pi]), np.array([-0.17, -0.05, 0])] ,
                # 1: [np.array([-0.17, -0.05, 0]), np.array([-0.17,-0.05,0]), np.array([0.17, -0.05, np.pi]), np.array([0.17, -0.05, np.pi]), np.array([0.17, -0.05, np.pi])]}

        # path = {0: [np.array([-0.17, -0.05, 0]),    np.array([0,-0.05,np.pi/2]),    np.array([0, 0.17, -np.pi/2]),  np.array([0, 0.17, -np.pi/2]), np.array([0,-0.05,np.pi]),
        #             np.array([-0.17, -0.05, 0]),    np.array([-0.17, -0.05, 0]),    np.array([-0.17, -0.05, 0]),    np.array([0, -0.05, 0]),       np.array([0,0.17,-np.pi/2])],
        #         1: [np.array([0, 0.17, -np.pi/2]),  np.array([0, 0.17, np.pi/2]),   np.array([0, 0.34, -np.pi/2]),  np.array([0, 0.34, -np.pi/2]), np.array([0, 0.34, -np.pi/2]),
        #             np.array([0, 0.34, -np.pi/2]),  np.array([0,-0.05,0]),          np.array([0.17,-0.05,np.pi]),   np.array([0.17,-0.05,np.pi]),  np.array([0.17,-0.05,np.pi])],
        #         2: [np.array([0.17, -0.05, np.pi]), np.array([0.17, -0.05, np.pi]), np.array([0.17, -0.05, np.pi]), np.array([-0.17, -0.05, 0]),   np.array([-0.34, -0.05, 0]),
        #             np.array([-0.34, -0.05, 0]),    np.array([-0.34, -0.05, 0]),    np.array([-0.34, -0.05, 0]),    np.array([-0.34, -0.05, 0]),   np.array([-0.17, -0.05, 0])] }

        print('Path: ', path)

        # Tell robots to follow their paths

        update_rate = rospy.Rate(1)  #tried slowing this down to give more buffer, but doesn't really help if the delay is in between steps
        for t in range(n_waypoints):
            print("Sending waypoints for step {}".format(t))

            ordering = path['o'][t]
            print(ordering)
            if len(ordering) == 0:  # Send all robots at once
                for r in range(n_rob):
                    print("Robot {}, go!".format(r))
                    pose_msg = FillPoseMsg(path[r][t])
                    self.waypoint_pubs[r].publish(pose_msg)

                # Wait for them to finish
                self.n_robots_done = 0
                while self.n_robots_done is not n_rob:
                    print("Waiting for robots to finish their waypoint")
                    update_rate.sleep()
                print("sent commands to all robots")
            else:
                for r in ordering:  # TODO change this
                    print("Robot {}, go!".format(r))
                    pose_msg = FillPoseMsg(path[r][t])
                    self.waypoint_pubs[r].publish(pose_msg)
                    self.n_robots_done = 0
                    while self.n_robots_done is not 1:
                       print("Waiting for robots to finish their waypoint")
                       update_rate.sleep()
            print("All robots done with step {}".format(t))
        print("All paths completed")

if __name__ == '__main__':
    rospy.init_node('mrdrrt_commander_node', anonymous=True)
    prmnode = MrdrrtCommanderNode()
    rospy.spin()
