import numpy as np
from Graph import Graph
from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from Tree import Tree

class MRdRRTPlanner(object):
    def __init__(self, prm, visualize=False):
        self.roadmap = prm.graph
        self.env = prm.env

    def Oracle(self,qnear,qrand):
        pass

    def LocalConnector(self):
        # Just do naïve local connector first, where you move the robots at
        # constant "phase"/alpha along their paths and check for collisions.
        pass

    def Expand(self,T):
        pass

    def ConnectToTarget(self,T,t):
        pass

    def RetrievePath(self,T,P):
        pass

    def FindPath(self, sconfigs, gconfigs):
        """Inputs: list of start and goal configs for robots."""
        self.T = Tree()
        # TODO put MRdRRT here
        pass
