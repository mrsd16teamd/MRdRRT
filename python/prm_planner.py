from simple_environment import SimpleEnvironment
from prm_graph import Graph

import cPickle as pickle
import numpy as np
import time
import sys


class PRMPlanner(object):
    """PRM planner node: interfaces with environment and graph structure.
    Can either generate a new roadmap or load a saved one.
    """

    def __init__(self, n_nodes=300, map_id=1, load=True, visualize=True, filepath=None):
        self.env = SimpleEnvironment(map_id, visualize)
        self.graph = Graph(self.env)
        self.n_nodes = n_nodes
        self.map_id = map_id

        self.visualize = visualize
        if self.visualize:
            self.env.InitializePlot()
        if load:
            if map_id == 1 and filepath is None:
                filepath = '../roadmaps/cube_center.p'
            elif map_id == 2 and filepath is None:
                filepath = '../roadmaps/t_map_simple.p'
            elif map_id == 3 and filepath is None:
                filepath = '../roadmaps/t_map_4node.p'
            self.LoadRoadmap(filepath)
        else:
            raw_input("Hit enter to generate and save new roadmap.")
            self.GenerateRoadmap()
            self.SaveRoadmap()

    def GenerateRoadmap(self):
        """Standard PRM algorithm."""
        print("Generating roadmap...")

        make_simple_prm = True

        if make_simple_prm:
            # node_configs = np.array([[-0.225+k*0.075, -0.05] for k in range(6)] + [[0, -0.05+k*0.075] for k in range(5)]) # 7.5cm resolution
            # node_configs = np.array([[-0.28+k*0.14, -0.05] for k in range(5)] + [[0, -0.05+k*0.14] for k in range(3)]) # 14cm resolution
            # node_configs = np.array([[-0.14+k*0.14, -0.05] for k in range(3)] + [[0, -0.05+k*0.14] for k in range(3)]) # 14cm resolution
            node_configs = np.array([[-0.30, -0.05], [-0.15, -0.05], [0.0, -0.05], [0.15, -0.05], [0.30, -0.05], [0, 0.15], [0, 0.30]])

            # node_configs = np.array([[-0.15, -0.05], [0.0, -0.05], [0.15, -0.05], [0.0, 0.15]])

            for node in node_configs:
                node_id = self.graph.AddVertex(node)

            # edges = [(0,0), (0,1), (1,0), (1,1), (1,2), (1,3), (2,1), (2,2), (3,1), (3,3)]
            # for e in edges:
            #     self.graph.AddEdge(e[0], e[1])

            # for id, config in enumerate(self.graph.vertices):
            #     n_ids, n_configs = self.graph.GetNeighbors(id, 0.20, 3)
            #     for i, n_id in enumerate(n_ids):
            #         if not self.env.CollisionOnLine(config, n_configs[i]):
            #             self.graph.AddEdge(id, n_id)

            for i, config_i in enumerate(self.graph.vertices):
                for j, config_j in enumerate(self.graph.vertices):
                    if not self.env.CollisionOnLine(config_i, config_j):
                        self.graph.AddEdge(i, j)

        else:
            while (len(self.graph.vertices) < self.n_nodes):
                # Generate random sample, check that's in Cfree
                qnew = self.env.SampleConfig()
                new_id = self.graph.AddVertex(qnew)

                # Get neighbors for new node, check for collision on edge and add to PRM
                n_ids, n_configs = self.graph.GetNeighbors(new_id)
                for i, n_id in enumerate(n_ids):
                    if not self.env.CollisionOnLine(qnew, n_configs[i]):
                        self.graph.AddEdge(new_id, n_id)


        if self.visualize:
            self.PlotRoadmap()

    def SaveRoadmap(self):
        """Save generated roadmap for future use, in pickle."""
        prm_graph = dict()
        prm_graph['vertices'] = self.graph.vertices
        prm_graph['edges'] = self.graph.edges

        if self.map_id == 1:
            filepath = '../roadmaps/cube_center.p'
        elif self.map_id == 2:
            filepath = '../roadmaps/t_map_simple.p'
        else:
            print("Enter a valid map ID.")
            sys.exit()
        with open(filepath, "wb") as f:
            pickle.dump(prm_graph, f)
            print("Saved roadmap.")

    def LoadRoadmap(self, filepath):
        """Loads pickle with pre-made roadmap."""
        print("Loading roadmap.")
        with open(filepath, 'rb') as f:
            prm_graph = pickle.load(f)
            self.graph.vertices = prm_graph['vertices']
            self.graph.edges = prm_graph['edges']
            print self.graph.vertices
            print self.graph.edges
            if self.visualize:
                self.PlotRoadmap()
                raw_input("Wait for plot and check roadmap.")

    def PlotRoadmap(self):
        """Plots roadmap's nodes and edges.
        Assumes that roadmap and pyplot figure are initialized.
        """
        print("Plotting roadmap..")
        for i, v in enumerate(self.graph.vertices):
            self.env.PlotPoint(v)
            for n in self.graph.edges[i]:
                self.env.PlotEdge(v, self.graph.vertices[n])

    def VisualizePath(self, path, edgecolor='g-'):
        """Plots final path on roadmap between two points, in different color.
        Assumes pyplot figure is initialized.
        """
        self.env.PlotPoint(path[0], 'g', 7)
        self.env.PlotPoint(path[-1], 'm', 7)
        for i, config in enumerate(path[0:-1]):
            self.env.PlotEdge(path[i], path[i+1], edgecolor, 2)

    def PostProcessPRMPath(self, point_path, sconfig, gconfig):
        """Post-processes path from roadmap before sending to Cozmo.
        Does two things: add angles to path, add start and goal configs
        For angles, just points robot towards next waypoint.
        """
        # TODO check collision along these edges? What to do if there is one?
        point_path.insert(0, sconfig[0:2])  # assume we don't need start pose in waypoint list
        point_path.append(gconfig[0:2])

        path_w_angles = []
        for i, point in enumerate(point_path[0:-1]):  # First point -> second to last
            vec2next = point_path[i+1] - point_path[i]
            angle = np.arctan2(vec2next[1], vec2next[0])
            config = np.append(point, angle)
            path_w_angles.append(config)

        path_w_angles.append(gconfig)
        return path_w_angles

    def FindPath(self, sconfig, gconfig):
        """Find nearest vertices to sconfig and gconfig
        raw_input should be in numpy arrays of dim 3 (x,y,theta)
        """
        sid = self.graph.GetNearestNode(sconfig[0:2])
        gid = self.graph.GetNearestNode(gconfig[0:2])
        start_angle = sconfig[2]
        goal_angle = gconfig[2]

        point_path = self.graph.Djikstra(sid, gid)
        if len(point_path) == 0:
            return []

        # Add angles and sconfig, gconfig
        path = self.PostProcessPRMPath(point_path, sconfig, gconfig)

        if self.visualize:
            self.VisualizePath(point_path)
        return path
