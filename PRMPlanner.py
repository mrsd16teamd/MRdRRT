import numpy as np
from Graph import Graph
from SimpleEnvironment import SimpleEnvironment
import time

class PRMPlanner(object):

    def __init__(self, planning_env, N=300, visualize=False):
        self.graph = Graph(planning_env)
        self.env = planning_env
        self.N = N
        self.visualize = visualize
        if self.visualize:
            self.env.InitializePlot()

    def GenerateRoadmap(self):
        while (len(self.graph.vertices) < self.N):
            # Genereate random sample, check that's in Cfree
            qnew = self.env.SampleConfig()
            if not self.env.CheckCollision(qnew):
                new_id = self.graph.AddVertex(qnew)

                if self.visualize:
                    self.env.PlotPoint(qnew)

                # Get neighbors for new node, check for collision on edge and add to PRM
                n_ids,n_configs = self.graph.GetNeighbors(new_id)
                for i, n_id in enumerate(n_ids):
                    if not self.env.CollisionOnLine(qnew, n_configs[i]):
                        self.graph.AddEdge(new_id, n_id)

                        if self.visualize:
                            self.env.PlotEdge(self.graph.vertices[new_id], self.graph.vertices[n_id])

    def VisualizePath(self, path):
        self.env.PlotPoint(path[0],'g')
        self.env.PlotPoint(path[-1],'y')
        for i,config in enumerate(path[0:-1]):
            #TODO make this linewidth thing actually work
            self.env.PlotEdge(path[i],path[i+1],'g-', 2.5)

    def FindPath(self, sconfig, gconfig):
        # Find nearest vertices to sconfig and gconfig
        sid = self.graph.GetNearestNode(sconfig)
        gid = self.graph.GetNearestNode(gconfig)

        path = self.graph.Djikstra(sid, gid)
        if len(path)==0:
            return []

        # TODO check collision along these edges? What to do if there is one?
        path.insert(0, sconfig)
        path.append(gconfig)
        if self.visualize:
            self.VisualizePath(path)
        return path
