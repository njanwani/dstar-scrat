import numpy as np

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