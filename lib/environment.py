import numpy as np
from lib.node import *
from tqdm import tqdm

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

    def __init__(self, pts, obstacles=[], mincost=1, nodetype=BasicNode):
        self.graph = {}
        self.obstacles = np.array(obstacles)
        self.pts = np.array(pts)
        self.mincost = mincost
        self.nodetype = nodetype


    def dist(self, v1, v2):
        return np.linalg.norm(v1.pt - v2.pt)
    

    def generate(self, extranodes=[]):
        """
        Generate environment graph
        """
        nodes = []
        for pt in self.pts:
            if np.array(pt) in self.obstacles:
                # print("Added obstacle:")
                # print(pt)
                nodes.append(self.nodetype(pt[0], pt[1], Node.OBSTACLE))
            else:
                nodes.append(self.nodetype(pt[0], pt[1]))

        for extra in extranodes:
            nodes.append(extra)
        
        self.graph = {}
        for v1 in tqdm(nodes):
            if v1 not in self.graph:
                self.graph[v1] = []

            for v2 in nodes:
                if self.dist(v1, v2) < self.mincost and v1 != v2:
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
        self.start = self.nodetype(start[0], start[1])
        self.goal = self.nodetype(goal[0], goal[1])
        self.generate(extranodes=[self.start, self.goal])
        self.add_points((start, goal), generate=False)

    def in_freespace(self):
        """
        ???
        """
        return super().in_freespace()

    def add_obstacle(self, obs_pts):
        """
        ???
        """
        return super().add_obstacle()
    
class OrderedList(Environment):

    def __init__(self, pts, obstacles=[], mincost=1):
        self.graph = {}
        self.obstacles = np.array(obstacles)
        self.pts = np.array(pts)
        self.mincost = mincost
        print(self.obstacles)

    def is_an_obstacle(self, pt):
        """
        Determine if given point is an obstacle
        """
        for obs in self.obstacles:
            if pt[0] == obs[0] and pt[1] == obs[1]:
                return True
        return False

    def generate(self, extranodes=[]):
        """
        Generate environment graph
        """
        nodes = []
        for pt in self.pts:
            if pt[0] == self.start_pos[0] and pt[1] == self.start_pos[1]:
                self.start = BasicNode(pt[0], pt[1])
                nodes.append(self.start)
            elif pt[0] == self.goal_pos[0] and pt[1] == self.goal_pos[1]:
                self.goal = BasicNode(pt[0], pt[1])
                nodes.append(self.goal)
            elif self.is_an_obstacle(pt):
                # print("Added obstacle:")
                # print(pt)
                nodes.append(BasicNode(pt[0], pt[1], Node.OBSTACLE))
            else:
                nodes.append(BasicNode(pt[0], pt[1]))

        
        self.graph = {}
        total_nodes = len(nodes)
        for i in range(0, total_nodes- 1):
            # print(nodes[i])
            self.graph[nodes[i]] = []
            if nodes[i+1].state != Node.OBSTACLE:
                self.graph[nodes[i]].append(nodes[i+1])
            if nodes[i-1].state != Node.OBSTACLE:
                self.graph[nodes[i]].append(nodes[i-1])
        
        self.graph[nodes[total_nodes-1]] = []
        if nodes[0].state != Node.OBSTACLE:
                self.graph[nodes[total_nodes-1]].append(nodes[0])
        if nodes[total_nodes-2].state != Node.OBSTACLE:
            self.graph[nodes[total_nodes-1]].append(nodes[total_nodes-2])


    
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
        Adds the start and goal (x,y) points to be saved as nodes
        """
        self.start_pos = (start[0], start[1])
        self.goal_pos = (goal[0], goal[1])
        self.generate()

    def in_freespace(self):
        """
        ???
        """
        return super().in_freespace()

    def add_obstacle(self, obs_pts):
        """
        ???
        """
        return super().add_obstacle()