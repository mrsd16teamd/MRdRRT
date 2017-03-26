from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from Graph import Graph
import numpy as np
import itertools

class ImplicitGraph(object):
    def __init__(self, env, roadmap, n_robots=2):
        self.roadmap = roadmap.graph
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
        neighbors_of_each = [] # list of lists
        for i in range(len(node)):
            neighbors_of_each.append(self.roadmap.edges[node[i]])

        # Return all possible combinations of neighbors
        neighbors = list(itertools.product(*neighbors_of_each))
        return neighbors

    def FindPath(self, sconfig, gconfig):
        # Find nearest vertices to sconfig and gconfig
        sid = self.graph.GetNearestNode(sconfig)
        gid = self.graph.GetNearestNode(gconfig)

if __name__ == "__main__":
    env = SimpleEnvironment()
    prm = PRMPlanner(N=300, load=True, filepath='prm_save.p', visualize=False)
    igraph = ImplicitGraph(env, prm)
    test = igraph.GetNeighbors([33, 100])
    import IPython
    IPython.embed()
