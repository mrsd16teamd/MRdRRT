# in ImplicitGraph
# RandomSample()
# NearestNeighbor(Tree,qrand)
# NearestNeighbors(Tree,goal,K) K=number of neighbors

from Graph import Graph
import numpy as np

class ImplicitGraph(object):
    def __init__(self, env, roadmap, n_robots=2):
        self.roadmap = roadmap
        self.env = env
        self.N = n_robots

    def RandomSample(self):
        c_config = []
        for i in range(self.N):
            c_config.append(self.env.SampleConfig())
        return np.array(c_config)

    def GetNeighbors(self,node):
        """
        Input: node as set of IDs for each PRM (list)
        Output: list of nodes (list of PRM node ids)
        """
        pass

    def NearestNeighbor(Tree, qrand):
        # TODO Move this to Tree?
        pass

    def NearestNeighbors(Tree, goal, K):
        pass
