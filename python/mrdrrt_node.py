#!/usr/bin/python
# -*- encoding: utf-8 -*-

import rospy
import tf
import rospkg

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from geometry_msgs.msg import (Twist, TransformStamped, PoseStamped)

import numpy as np
from prm_planner import PRMPlanner
from mrdrrt_planner import MRdRRTPlanner


n_rob = 2
gconfigs = np.array([[5, 30], [15, 35]]) #hard-coded goal configs, for now

class MrdrrtCommanderNode:

    def __init__(self, *args):
        rospack = rospkg.RosPack()
        path = rospack.get_path('mrdrrt')
        map_path = path + '/roadmaps/' + 't_map_prm.p'

        self.prm = PRMPlanner(n_nodes=300, map_id=map_id, load=True, visualize=False, filepath=map_path)
        self.mrdrrt = MRdRRTPlanner(self.prm, n_robots=n_rob, visualize=False)

        self.map_frame = '/map'

        self.robot_namespaces = ['cozmo'+str(i) for i in range(n_rob)]
        self.robot_frames = [self.robot_namespaces[i] + '/base_link' for i in range(n_rob)]
        self.waypoint_pubs = [rospy.Publisher(self.robot_namespaces[i]+'/goal', PoseStamped, queue_size=3) for i in range(n_rob)]

        self.tf_listener = tf.TransformListener()
        self.plan_serv = rospy.Service('mrdrrt_start', Empty, self.PlanPath)

        # TODO define subscriber for checking if robots are done going to waypoint
        self.robots_done = rospy.Subscriber('/goal_reached', std_msgs.msg.Int8, queue_size=5)


    def GetRobotPose(self, robot_id):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frames[robot_id], rospy.Time(0))
            eul = tf.transformations.euler_from_quaternion(rot)
            yaw = eul[2]
            config = np.array([trans[0], trans[1], yaw])
            return config
        except:
            print("Couldn't get transform between " + self.map_frame + " and " + self.robot_frames[robot_id])
            return False

    def PlanPath(self, request):
        # Find robots' starting configurations
        sconfigs = []
        for i in range(n_rob):
            start_configs.append(self.GetRobotPose(i))
        sconfigs = np.array(sconfigs)

        # Get path from mrdrrt - will dictionary of lists of numpy arrays
        path = self.mrdrrt.FindPath(sconfigs, gconfigs)


