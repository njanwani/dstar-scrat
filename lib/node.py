import numpy as np

class Node:
    FREE = 0
    OBSTACLE = 1
    def __init__(self, x, y):
        raise NotImplemented


    def cost_to(self, node):
        raise NotImplemented
    

class BasicNode:
    """
    Basic x,y node. Changing to a more complex state will likely require changes in other lib files
    """

    def __init__(self, x, y, variant=None):
        self.x = x
        self.y = y
        self.pt = np.array([x, y])
        if variant == None:
            self.variant = Node.FREE
        else:
            self.variant = variant


    def cost_to(self, node):
        """
        Cost function
        """
        if self.variant == Node.FREE and node.variant == Node.FREE:
            return np.linalg.norm(self.pt - node.pt)
        elif self.variant == Node.OBSTACLE or node.variant == Node.OBSTACLE:
            return float('inf')
        else:
            Exception('Unknown edge configuration')
    

    def nodes_to_xy(path):
        """
        Converts nodes to xy path (only works for BasicNode)
        """
        pointpath = []

        for node in path:
            pointpath.append([node.x, node.y])

        return pointpath
    

    def make_obstacle(self):
        self.variant = Node.OBSTACLE
    

    def __str__(self):
        return f'({self.x}, {self.y})'