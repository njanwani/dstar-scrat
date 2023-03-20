import numpy as np

class Node:
    FREE = 0
    OBSTACLE = 1

    UNPROCESSED = 2
    DONE = 3
    ONDECK = 4
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
        self.state = Node.UNPROCESSED
        if variant == None:
            self.variant = Node.FREE
        else:
            self.variant = variant
        self.creach = 0.0
        self.cost = 0.0
        self.parent = None


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
    
    
    def make_free(self):
        self.variant = Node.FREE

    def __str__(self):
        return f'({self.x}, {self.y})'
    
    def __lt__(self, other):
        """
        Define less-than so we can organize states by cost
        """
        return self.cost < other.cost