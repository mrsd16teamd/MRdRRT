import numpy as np
from Graph import Graph
from SimpleEnvironment import SimpleEnvironment
import time
import cPickle as pickle

class PRMPlanner(object):

    def __init__(self, N=300, load=True, visualize=False, filepath=None):
        self.env = SimpleEnvironment()
        self.graph = Graph(self.env)
        self.N = N
        self.visualize = visualize
        if load:
            self.LoadRoadmap(filepath)
        if self.visualize:
            self.env.InitializePlot()

    def GenerateRoadmap(self):
        while (len(self.graph.vertices) < self.N):
            # Genereate random sample, check that's in Cfree
            qnew = self.env.SampleConfig()
            new_id = self.graph.AddVertex(qnew)

            # Get neighbors for new node, check for collision on edge and add to PRM
            n_ids,n_configs = self.graph.GetNeighbors(new_id)
            for i, n_id in enumerate(n_ids):
                if not self.env.CollisionOnLine(qnew, n_configs[i]):
                    self.graph.AddEdge(new_id, n_id)

        if self.visualize:
            self.PlotRoadmap()

    def SaveRoadmap(self):
        prm_graph = dict()
        prm_graph['vertices'] = self.graph.vertices
        prm_graph['edges'] = self.graph.edges
        pickle.dump(prm_graph, open( "prm_save.p", "wb" ) )

    def LoadRoadmap(self, filepath='prm_save.p'):
        print("Loading roadmap.")
        prm_graph = pickle.load( open(filepath, 'rb') )
        self.graph.vertices = prm_graph['vertices']
        self.graph.edges = prm_graph['edges']

    def PlotRoadmap(self):
        print("Plotting roadmap..")
        for i,v in enumerate(self.graph.vertices):
            self.env.PlotPoint(v)
            for n in self.graph.edges[i]:
                self.env.PlotEdge(v,self.graph.vertices[n])

    def VisualizePath(self, path):
        self.env.PlotPoint(path[0],'g',7)
        self.env.PlotPoint(path[-1],'y',7)
        for i,config in enumerate(path[0:-1]):
            #TODO make this linewidth thing actually work
            self.env.PlotEdge(path[i],path[i+1],'g-', 2)

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
