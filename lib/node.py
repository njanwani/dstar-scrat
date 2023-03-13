import numpy as np

class Node:

    def __init__(self, x, y):
        raise NotImplemented


    def cost_to(self, node):
        raise NotImplemented
    

class BasicNode:
    """
    Basic x,y node. Changing to a more complex state will likely require changes in other lib files
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.pt = np.array([x, y])


    def cost_to(self, node):
        """
        Cost function
        """
        return np.linalg.norm(self.pt - node.pt)
    

    def nodes_to_xy(path):
        """
        Converts nodes to xy path (only works for BasicNode)
        """
        pointpath = []

        for node in path:
            pointpath.append([node.x, node.y])

        return pointpath
    

    def __str__(self):
        return f'({self.x}, {self.y})'