import numpy as np
from Graph import Graph
from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from Tree import Tree
from ImplicitGraph import ImplicitGraph

class MRdRRTPlanner(object):
    def __init__(self, prm, visualize=False):
        self.env = prm.env
        self.implicitgraph = ImplicitGraph()
        self.tree = Tree()  # Tree that grows from starting configuration

    def Oracle(self,qnear,qrand):
        """
        Given randomly sampled comp config and nearest config on current tree,
        return qnew, a neighbor of qnear on the implicit graph that hasn't been
        explored yet, and is closest (by some metric, TBD) to qnear.
        """
        pass

    def LocalConnector(self, config1, config2):
        """
        Given two composite configurations, check if collision free movement
        between them is possible. If coordinated movement/ordering is required,
        return ordering of robots as list.
        Doing naive implementation first, where robots are moved along path by
        constant phase/alpha from start to end configs and check collision at
        each point.
        Later, do more complicated one.
        """
        pass

    def Expand(self,T):
        pass

    def ConnectToTarget(self,T,t):
        """
        Called at the end of each iteration.
        Check if it's possible to get to goal from closest nodes in current tree.
        """
        pass

    def RetrievePath(self,T,P):
        """
        Called when a collision-free path to goal config is found.
        Returns final path thru implicit graph to get from start to goal. s
        """
        pass

    def FindPath(self, sconfigs, gconfigs):
        """
        Inputs: list of start and goal configs for robots.
        """
        self.T = Tree()
        # TODO put MRdRRT here
        pass
