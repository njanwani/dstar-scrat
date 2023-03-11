import numpy as np
from node import *

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

    def __init__(self, N):
        self.N = N
        self.graph = None
        self.obstacles = None
        self.pts = None

    def generate(self):
        # generate graph and plot astar path
        nodes = []
        for pt in self.pts:
            nodes.append(BasicNode(pt[0], pt[1]))

        self.graph = {}
        for v1 in nodes:
            self.graph[v1] = []
            for v2 in nodes:
                if v1.cost_to(v2) < 1 and v1 != v2:
                    self.graph[v1].append(v2)

    
    def add_points(self, newpts):
        self.pts = np.append(self.pts, newpts, axis=0)

    
    def add_startgoal(self, start, goal):
        raise NotImplemented
    
    def in_freespace(self):
        return super().in_freespace()
    
