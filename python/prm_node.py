#!/usr/bin/env python
import rospy
import tf

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mrdrrt.srv import PrmSrv

import numpy as np
from prm_planner import PRMPlanner

import rospkg

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
        if map_id == 1:
            filename = 'cube_center.p'
        elif map_id == 2:
            filename = 't_map_prm.p'
        map_path = path + '/roadmaps/' + filename

        self.prm = PRMPlanner(n_nodes=1000, map_id=map_id, load=True, visualize=False, filepath = map_path)
        self.tf_listener = tf.TransformListener()

        self.plan_pub = rospy.Publisher('prm_path', Path, queue_size=1)
        self.plan_serv = rospy.Service('prm_plan', PrmSrv, self.PlanPath)

        self.map_frame = 'world'
        self.robot_frame = 'base_link'

        print("Ready to serve! Call prm_plan service with goal pose.")

    def PlanPath(self, request):
        """Processes service request (query for path to goal point).
        Path is published in nav_msgs/Path type.
        """
        print("Starting PlanPath.")
        goal_config = np.array([request.goal_pose.x, request.goal_pose.y, request.goal_pose.theta])

        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, rospy.Time(0))
            eul = tf.transformations.euler_from_quaternion(rot)
            yaw = eul[2]
            # trans = [-30, -30]
            # yaw = 0.1
        except:
            print("Couldn't get transform between " + self.map_frame + " and " + self.robot_frame)
            return False


        start_config = np.array([trans[0], trans[1], yaw])

        prm_path = self.prm.FindPath(start_config, goal_config)

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
            for config in prm_path:
                pose = PoseStamped()
                pose.pose.position.x = config[0]
                pose.pose.position.y = config[1]

                # TODO test this
                yaw = config[2]
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                pub_path.append(pose)
                print(config)

            plan_msg.poses = pub_path
            self.plan_pub.publish(plan_msg)

        print("Published path to prm_path.")
        return True

if __name__ == '__main__':
    rospy.init_node('prm_planner_node', anonymous=True)
    prmnode = PRMPlannerNode()
    rospy.spin()
