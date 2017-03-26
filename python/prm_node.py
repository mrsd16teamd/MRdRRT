#!/usr/bin/env python
import numpy as np
import rospy
import geometry_msgs.msg
import nav_msgs.msg
from nav_msgs.msg import Path
import roslaunch
import tf
from PRMPlanner import PRMPlanner
# from mrdrrt.srv import PrmSrv

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

        path = self.prm.PlanPath(start_config, goal_config)

        #TODO process path and publish as nav_msgs/Path
        # - header
        # - geometry_msgs/PoseStamped[] poses

if __name__ == '__main__':
    prmnode = PRMPlannerNode()
    rospy.spin()
