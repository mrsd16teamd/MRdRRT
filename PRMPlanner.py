import numpy as np
from Graph import Graph
from SimpleEnvironment import SimpleEnvironment
import time

class PRMPlanner(object):

    N = 100  #Number of points desired
    visualize = True

    def __init__(self, planning_env):
        self.graph = Graph(planning_env)
        self.env = planning_env
        self.env.InitializePlot()

    def GenerateRoadmap(self):
        while (len(self.graph.vertices) < self.N):
            # Genereate random sample, check that's in Cfree
            qnew = env.SampleConfig()
            if not env.CheckCollision(qnew):
                new_id = self.graph.AddVertex(qnew)

                if (self.visualize):
                    self.env.PlotPoint(qnew)

                # Get neighbors for new node, check for collision on edge and add to PRM
                n_ids,n_configs = self.graph.GetNeighbors(qnew)
                for i, n_id in enumerate(n_ids):
                    if not self.env.CollisionOnLine(qnew, n_configs[i]):
                        self.graph.AddEdge(new_id, n_id)

                        if (self.visualize):
                            self.env.PlotEdge(self.graph.vertices[new_id], self.graph.vertices[n_id])

    def FindPath(self, sconfig, gconfig):
        # Find nearest vertices to

        # TODO put djikstra here
        path = []

        pass


if __name__ == "__main__":
    env = SimpleEnvironment()
    prm = PRMPlanner(env)
    prm.GenerateRoadmap()
    raw_input("Done making roadmap.")
