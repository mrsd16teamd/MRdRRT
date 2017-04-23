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


n_rob = 2
gconfigs = np.array([[5, 30], [15, 35]]) #hard-coded goal configs, for now

class MrdrrtCommanderNode:

    def __init__(self, *args):
        rospack = rospkg.RosPack()
        path = rospack.get_path('mrdrrt')
        map_path = path + '/roadmaps/' + 't_map_prm.p'

        # self.prm = PRMPlanner(n_nodes=300, map_id=map_id, load=True, visualize=False, filepath=map_path)
        # self.mrdrrt = MRdRRTPlanner(self.prm, n_robots=n_rob, visualize=False)

        self.map_frame = '/map'

        self.robot_namespaces = ['cozmo'+str(i) for i in range(n_rob)]
        self.robot_frames = [self.robot_namespaces[i] + '/base_link' for i in range(n_rob)]
        self.waypoint_pubs = [rospy.Publisher(self.robot_namespaces[i]+'/goal', PoseStamped, queue_size=3) for i in range(n_rob)]

        self.tf_listener = tf.TransformListener()
        self.plan_serv = rospy.Service('mrdrrt_start', Trigger, self.PlanPath)

        # TODO define subscriber for checking if robots are done going to waypoint
        self.robots_done_sub = rospy.Subscriber('/goal_reached', std_msgs.msg.Int8, self.GoalReachedCb, queue_size=5)

        print("MRdRRT commander node ready!")

    def GoalReachedCb(self, msg):
        self.n_robots_done += 1

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
            sconfigs.append(self.GetRobotPose(i))
        sconfigs = np.array(sconfigs)

        # Get path from mrdrrt - will dictionary of lists of numpy arrays
        # path = self.mrdrrt.FindPath(sconfigs, gconfigs)

        path = {0: [np.array([0.1, -0.05,0]), np.array([0,0,0])], 1: [np.array([-0.2, -0.05, 0]), np.array([-0.2,0.2,0])]}

        # Tell robots to follow their paths
        n_waypoints = len(path[0])
        for t in range(n_waypoints):
            # Send robot their waypoints
            for r in range(n_rob):
                pose_msg = PoseStamped()
                pose_msg.pose.position.x, pose_msg.pose.position.y = path[r][t][0], path[r][t][1]

                quat = tf.transformations.quaternion_from_euler(0, 0, path[r][t][2])
                pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
                
                self.waypoint_pubs[r].publish(pose_msg)

            # Wait for them to finish
            self.n_robots_done = 0
            while self.n_robots_done is not n_rob:
                sleep(1)
            print("All robots done with their waypoints.")


if __name__ == '__main__':
    rospy.init_node('mrdrrt_commander_node', anonymous=True)
    prmnode = MrdrrtCommanderNode()
    rospy.spin()

