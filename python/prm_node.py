#!/usr/bin/python
# -*- encoding: utf-8 -*-

import rospy
import tf
import rospkg

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mrdrrt.srv import PrmSrv
from transformations import (euler_from_quaternion, quaternion_from_euler)

import numpy as np
from prm_planner import PRMPlanner


map_id = 1

class PRMPlannerNode(object):
    """PRM node for single Cozmo robot.
    This node will listen for service calls from a robot for a goal state
    We will then find the current state of the robot, and use a PRM to
    """

    def __init__(self):
        """ Initializes its own PRM roadmap. Assume map matches real life"""

        rospack = rospkg.RosPack()
        path = rospack.get_path('mrdrrt')
        map_path = path + '/roadmaps/t_map_prm.p'

        self.prm = PRMPlanner(n_nodes=300, map_id=map_id, load=True, visualize=False, filepath = map_path)

        self.ns = rospy.get_namespace()[:-1]
        self.map_frame = '/map'
        self.robot_frame = self.ns + '/base_link'

        self.tf_listener = tf.TransformListener()
        self.plan_pub = rospy.Publisher(self.ns+'/path', Path, queue_size=1)

        self.plan_serv = rospy.Service('prm_plan', PrmSrv, self.service_call)
        self.goal_sub = rospy.Subscriber('goal', PoseStamped, self.goal_callback, queue_size=1)

        print("Ready to serve! Call prm_plan service with goal pose.")

    def GetRobotPose(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, rospy.Time(0))
            eul = tf.transformations.euler_from_quaternion(rot)
            yaw = eul[2]
            config = np.array([trans[0], trans[1], yaw])
            return config
        except:
            print("Couldn't get transform between " + self.map_frame + " and " + self.robot_frame)
            return False

    def service_call(self, request):
        goal_config = np.array([request.goal_pose.x, request.goal_pose.y, request.goal_pose.theta])
        self.PlanPath(goal_config)

    def goal_callback(self, msg):
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        q = msg.pose.orientation
        goal_th = euler_from_quaternion(np.array([q.x,q.y,q.z,q.w]))[2]
        goal_config = np.array([goal_x, goal_y, goal_th])
        self.PlanPath(goal_config)

    def PlanPath(self, goal_config):
        """Processes service request (query for path to goal point).
        Path is published in nav_msgs/Path type.
        """
        print("Starting PRM PlanPath.")

        start_config = self.GetRobotPose()

        prm_path = self.prm.FindPath(start_config, goal_config)

        # Dummy path
        # corner1 = np.array([0.2, 0, -np.pi/2])
        # corner2 = np.array([0.2, -0.2, np.pi])
        # corner3 = np.array([0, -0.2, np.pi/2])
        # prm_path = [corner1, corner2, corner3 , goal_config]

        # Make sure start_config is not in path!

        # Process path and publish as nav_msgs/Path
        # - header
        # - geometry_msgs/PoseStamped[] poses
        if len(prm_path) != 0:
            plan_msg = Path()

            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            plan_msg.header = h

            pub_path = []
            print("Path: ")
            for config in prm_path[1:]: #ignore first node in path; it's the start config
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = config[0], config[1]

                quat = tf.transformations.quaternion_from_euler(0, 0, config[2])
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
                pub_path.append(pose)
                print(pose.pose.position.x, pose.pose.position.y, config[2])

            plan_msg.poses = pub_path
            self.plan_pub.publish(plan_msg)

        print("Published path to prm_path.")
        return True

if __name__ == '__main__':
    rospy.init_node('prm_planner_node', anonymous=True)
    prmnode = PRMPlannerNode()
    rospy.spin()
