import numpy as np
import queue

class Planner:

    def __init__():
        raise NotImplemented
    
    def plan():
        raise NotImplemented
    

class Node:

    def __init__(self, x, y):
        raise NotImplemented


    def cost_to(self, node):
        raise NotImplemented
    

class BasicNode:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.pt = np.array([x, y])


    def cost_to(self, node):
        return np.linalg.norm(self.pt - node.pt)
    

    def __str__(self):
        return f'({self.x}, {self.y})'

class Astar(Planner):

    def __init__(self):
        self.path = []

    def plan(self, graph, start, goal):
        pq = queue.PriorityQueue()
        for node in graph[start]:
            pq.put((start.cost_to(node), [start, node]))

        distances = {}

        while not pq.empty():
            dist, path = pq.get()

            if path[-1] not in distances:
                distances[path[-1]] = dist
            elif distances[path[-1]] > dist:
                distances[path[-1]] = dist
            elif distances[path[-1]] < dist:
                continue
                
            # print('current', path)
            if path[-1] == goal:
                self.path = path
                return path
            
            for node in graph[path[-1]]:
                if node not in path:
                    pq.put((dist + path[-1].cost_to(node) + node.cost_to(goal), path + [node]))

        print('FAILED')
        return None
