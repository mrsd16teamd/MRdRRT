from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from Graph import Graph
import numpy as np

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
        # TODO test this
        neighbors_of_each = [] # list of lists - neig
        for i in len(node):
            neighbors_of_each.append(self.roadmap.edges[node[i]])
        neighbors = list(itertools.product(*neighbors_of_each))

    def NearestNeighbor(Tree, qrand):
        # TODO Move this to Tree?
        pass

    def NearestNeighbors(Tree, goal, K):
        pass

    def FindPath(self, sconfig, gconfig):
        # Find nearest vertices to sconfig and gconfig
        sid = self.graph.GetNearestNode(sconfig)
        gid = self.graph.GetNearestNode(gconfig)

if __name__ == "__main__":
    env = SimpleEnvironment()
    prm = PRMPlanner(env, N=300, load=True, visualize=False)
    ImplicitGraph(env, prm)

    import IPython
    IPython.embed()
