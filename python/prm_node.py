#!/usr/bin/env python
import rospy
import tf

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
# from mrdrrt.srv import PrmSrv

import numpy as np
from PRMPlanner import PRMPlanner

# This node will listen for service calls from a robot for a goal state
# We will then find the current state of the robot, and use a PRM to

class PRMPlannerNode(object):
    def __init__(self):
        self.prm = PRMPlanner(N=300, load=True, visualize=False, filepath='/home/kazu/cozmo_ws/src/MRdRRT/python/prm_save.p')
        self.listener = tf.TransformListener()

        rospy.init_node('prm_planner_node', anonymous=True)
        self.plan_pub = rospy.Publisher('prm_path', Path, queue_size=1)

        self.plan_serv = rospy.Service('prm_plan', mrdrrt.srv.PrmSrv, self.PlanPath)   # TODO fix this

    def PlanPath(self, req):
        goal_config = np.array([req.x, req.y])
        (trans,rot) = listener.lookupTransform('/world', '/base_link', rospy.Time(0))
        start_config = np.array([trans[0], trans[1]])

        prm_path = self.prm.PlanPath(start_config, goal_config)

        #Process path and publish as nav_msgs/Path
        # - header
        # - geometry_msgs/PoseStamped[] poses
        # TODO test this
		if len(path)!=0:
            plan_msg = Path()

            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            plan_msg.header = h

            for config in prm_path:
                pose = PoseStamped()
                pose.x = config[0]
                pose.y = config[1]
                pose.theta = 0  # Do we need this?
                Path.poses.append(pose)

            plan_pub.publish(plan_msg)


if __name__ == '__main__':
    prmnode = PRMPlannerNode()
    rospy.spin()
