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
from PRMPlanner import PRMPlanner

# This node will listen for service calls from a robot for a goal state
# We will then find the current state of the robot, and use a PRM to

class PRMPlannerNode(object):
    def __init__(self):
        self.prm = PRMPlanner(N=300, load=True, visualize=False, filepath='/home/kazu/cozmo_ws/src/MRdRRT/python/prm_save.p')
        self.tf_listener = tf.TransformListener()

        self.plan_pub = rospy.Publisher('prm_path', Path, queue_size=1)
        self.plan_serv = rospy.Service('prm_plan', PrmSrv, self.PlanPath)

        self.map_frame = '/world'
        self.robot_frame = '/base_link'

        print("Ready to serve!.")

    def PlanPath(self, req):
        print("Starting PlanPath.")
        goal_config = np.array([req.goal_pose.x, req.goal_pose.y])

        try:
            # (trans,rot) = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, rospy.Time(0))
            trans = [-30, -30]
        except:
            print("Couldn't get transform between " + self.map_frame + " and " + self.robot_frame)
            return False

        start_config = np.array([trans[0], trans[1]])

        prm_path = self.prm.FindPath(start_config, goal_config)

        #Process path and publish as nav_msgs/Path
        # - header
        # - geometry_msgs/PoseStamped[] poses
        if len(prm_path)!=0:
            plan_msg = Path()

            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            plan_msg.header = h

            pub_path = []
            for config in prm_path:
                pose = PoseStamped()
                pose.pose.position.x = config[0]
                pose.pose.position.y = config[1]
                pub_path.append(pose)

            plan_msg.poses = pub_path
            self.plan_pub.publish(plan_msg)

        print("Published path to prm_path.")
        return True


if __name__ == '__main__':
    rospy.init_node('prm_planner_node', anonymous=True)
    prmnode = PRMPlannerNode()
    rospy.spin()
