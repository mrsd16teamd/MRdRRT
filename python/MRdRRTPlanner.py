import numpy as np
from Graph import Graph
from SimpleEnvironment import SimpleEnvironment
from PRMPlanner import PRMPlanner
from Tree import Tree
from ImplicitGraph import ImplicitGraph

"""
Notes:
- tree will have nodes of type np array (matrix)
"""


class MRdRRTPlanner(object):
    def __init__(self, prm, n_robots=2, visualize=False):
        self.n_robots = n_robots
        self.env = prm.env
        self.implicitgraph = ImplicitGraph(self.env, prm, n_robots)
        self.tree = Tree(self.env, self.implicitgraph)  # Tree that grows from starting configuration
        self.max_iter = 1000
        self.prm = prm #Here just for VisualizePlot

    def Oracle(self,qnear,qrand):
        """
        Given randomly sampled comp config and nearest config on current tree,
        return qnew, a neighbor of qnear on the implicit graph that hasn't been
        explored yet, and is closest (by sum of euclidean distances) to qnear.
        """
        neighbors = self.implicitgraph.GetNeighbors(qnear)

        min_dist = float("inf")
        nearest = None
        for node in neighbors:
            config = self.implicitgraph.NodeIdsToConfigs(node)
            dist = self.implicitgraph.ComputeCompositeDistance(config, qrand)
            if(dist<min_dist):
                min_dist = dist
                nearest = node
        return nearest

    def Expand(self):
        """
        Takes random sample and tries to expand tree in direction of sample.
        """
        qrand = self.implicitgraph.RandomSample()
        qnear, near_id = self.tree.NearestNeighbors(qrand,1)

        qnew  = self.Oracle(qnear, qrand)
        if qnew not in self.tree.vertices:
            new_id = self.tree.AddVertex(qnew)
            self.tree.AddEdge(near_id,new_id)

    def LocalConnector(self, config1, config2):
        """
        Given two composite configurations, check if collision free movement
        between them is possible. If coordinated movement/ordering is required,
        return ordering of robots as list.
        Doing naive implementation first, where robots are moved along path by
        constant phase/alpha from start to end configs and check collision at
        each point.
        Later, do more complicated one.
        Input: lists of node IDs
        """
        n_steps = 10
        steps = [i/float(n_steps) for i in range(n_steps+1)]

        for step in steps:
            q = config1 + (config2-config1)*step
            collision = self.env.CheckCollisionMultiple(q)
            if collision:
                return False #couldn't connect

        return True #Connection succeeded!

    def ConnectToTarget(self,gids):
        """
        Called at the end of each iteration.
        Check if it's possible to get to goal from closest nodes in current tree.
        Input: list of goal configurations
        """
        # Only checking closest node right now because that's all nearestneighbors does
        # for q in self.tree.NearestNeighbors(goal,1):
        g_config = self.implicitgraph.NodeIdsToConfigs(gids)
        neighbor, nid = self.tree.NearestNeighbors(g_config,1)
        n_config = self.implicitgraph.NodeIdsToConfigs(neighbor)
        success = self.LocalConnector(n_config,g_config)
        return success, nid

    def ConstructPath(self,neighbor_of_goal,sconfigs, gconfigs, sids, gids):
        """
        Called when a collision-free path to goal config is found.
        Returns final path thru implicit graph to get from start to goal.
        Inputs:
            neighbor_of_goal: (node IDs) node that was successfully connected to goal
            gconfigs: list of configurations of final goal
        """
        path = [gconfigs]
        path.append(self.implicitgraph.NodeIdsToConfigs(gids))

        #Follow pointers to parents from goal to reconstruct path, reverse
        node_id = neighbor_of_goal
        while (node_id in self.tree.edges.keys()):    #TODO find better end condition
            node = self.tree.vertices[node_id]
            path.append(self.implicitgraph.NodeIdsToConfigs(node))
            node_id = self.tree.edges[node_id]

        path.append(self.implicitgraph.NodeIdsToConfigs(sids))
        path.append(sconfigs)
        path.reverse()
        return path

    def VisualizePath(self, path):
        colors = ['k-','y-','g-']
        self.env.InitializePlot()
        self.prm.PlotRoadmap()
        for robot in range(len(path[0])):
            robot_path = []
            for i in range(len(path)):
                robot_path.append(path[i][robot,:])
            self.prm.VisualizePath(robot_path,colors[robot])
        raw_input("Check paths")

    def FindPath(self, sconfigs, gconfigs):
        """
        Inputs: list of start and goal configs for robots.
        """
        print("Looking for a path...")
        sids = self.implicitgraph.NearestNodeInGraph(sconfigs)
        gids = self.implicitgraph.NearestNodeInGraph(gconfigs)

        # Put start config in tree
        self.tree.AddVertex(sids)

        i = 0
        while (i<self.max_iter):
            self.Expand()
            success, nid = self.ConnectToTarget(gids)
            if success:
                print("Found a path! Constructing final path now..")
                path = self.ConstructPath(nid, sconfigs, gconfigs, sids, gids)
                break

            i += 1
            if(i%100 ==0):
                print(i)

        #TODO reconstruct path and add start, goal configs
        self.VisualizePath(path)




        # #DEBUG STUFF
        # self.env.InitializePlot()
        # self.prm.PlotRoadmap()
        # self.env.PlotPoint(sconfigs[0],'g',7)
        # self.env.PlotPoint(sconfigs[1],'g',7)
        # self.env.PlotPoint(gconfigs[0],'y',7)
        # self.env.PlotPoint(gconfigs[1],'y',7)
        # s_close = self.implicitgraph.NodeIdsToConfigs(sids)
        # g_close = self.implicitgraph.NodeIdsToConfigs(gids)
        # self.env.PlotPoint(s_close[0],'r',7)
        # self.env.PlotPoint(s_close[1],'r',7)
        # self.env.PlotPoint(g_close[0],'m',7)
        # self.env.PlotPoint(g_close[1],'m',7)
        # raw_input("wtf")
        # ####
