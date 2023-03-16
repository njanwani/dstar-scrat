import numpy as np
from lib.node import *

class Environment:
    
    def __init__(self):
        raise NotImplemented

    def generate():
        raise NotImplemented
    
    def add_obstacle(pts):
        raise NotImplemented
    
    def in_freespace(self):
        pass
    

class Uniform(Environment):

    def __init__(self, pts, mincost=1):
        self.graph = {}
        self.obstacles = None
        self.pts = np.array(pts)
        self.mincost = mincost

    def generate(self, extranodes=[]):
        """
        Generate environment graph
        """
        nodes = []
        for pt in self.pts:
            nodes.append(BasicNode(pt[0], pt[1]))

        for extra in extranodes:
            nodes.append(extra)
        
        self.graph = {}
        for v1 in nodes:
            if v1 not in self.graph:
                self.graph[v1] = []

            for v2 in nodes:
                if v1.cost_to(v2) < self.mincost and v1 != v2:
                    self.graph[v1].append(v2)


    
    def add_points(self, newpts, generate=True):
        """
        Add points (put x,y in iterable list)

        generate=True will generate a new graph
        """
        self.pts = np.append(self.pts, newpts, axis=0)
        if generate:
            self.generate()

    
    def add_startgoal(self, start, goal):
        """
        Adds the start and goal (x,y) points and saves them as nodes
        """
        self.start = BasicNode(start[0], start[1])
        self.goal = BasicNode(goal[0], goal[1])
        self.generate(extranodes=[self.start, self.goal])
        self.add_points((start, goal), generate=False)

    def in_freespace(self):
        """
        ???
        """
        return super().in_freespace()

    def add_obstacle(pts):
        """
        ???
        """
        return super().add_obstacle()
    
