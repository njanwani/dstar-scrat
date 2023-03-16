import numpy as np
from queue import PriorityQueue
    
class Planner:

    def __init__():
        raise NotImplemented
    
    def plan():
        raise NotImplemented

class Astar(Planner):

    def __init__(self):
        self.path = []

    def plan(self, graph, start, goal):
        """
        Classic (probably inefficient) Astar planner. Gets the job done.
        """

        pq = PriorityQueue()
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
                
            if path[-1] == goal:
                self.path = path
                return path
            
            for node in graph[path[-1]]:
                if node not in path:
                    pq.put((dist + path[-1].cost_to(node) + node.cost_to(goal), path + [node]))

        print('FAILED')
        return None
    

class Dstar(Planner):
    NEW = 0
    OPEN = 1
    CLOSED = 2

    def __init__(self, g, graph):
        """
        Initialization function (many details are left out and can be seen in
        the D* paper) that declares variables and initializes dictionaries.
        
        g       is the goal
        graph   is the environment graph dictionary 
        """
        self.t = {}                         # the types for a state X (i.e. NEW,
                                            # OPEN, or CLOSED)

        self.h = {}                         # the current ESTIMATES (i.e. robot
                                            # may find it is wrong) of the arc
                                            # costs from goal to X

        self.k = {}                         # decides whether we are a RAISE, 
                                            # LOWER, or Dijkstra "mode". these
                                            # are the priorities used to order
                                            # the pqueue. This is the
                                            # simplification made by Gunters
                                            # approach
        
        self.b = {}                         # backpointers dictionary to later
                                            # reconstruct the path. Very
                                            # important for D*...

        self.open_list = PriorityQueue()    # OPEN list of states

        self.path = []                      # will eventually hold the path

        self.graph = graph                  # graph with neighbor relations

        # initialization steps
        for x in graph.keys():
            self.t[x] = Dstar.NEW

        self.goal = g
        self.h[x] = 0
        self.open_list.put((self.h[x], self.goal))
        self.t[x] = Dstar.OPEN
        self.b[self.goal] = None

    def min_state(self):
        """
        Returns the top of the OPEN priority queue
        """
        return self.open_list.get()
    

    def get_kmin(self):
        """
        Peeks at the top of the OPEN priority queue 
        """
        pass # not sure what to write here since we get this in min_state()...


    def insert(self, x, h_new):
        """
        Inserts states into the open list based on their RAISE, LOWER, or 
        'Dijkstra' state
        """
        if self.t[x] == Dstar.NEW:
            self.k[x] = h_new
        elif self.t[x] == Dstar.OPEN:
            self.k[x] = min(self.k[x], h_new)
        elif self.t[x] == Dstar.CLOSED:
            self.k[x] = min(self.h[x], h_new)

        self.h[x] = h_new
        self.t[x] = Dstar.OPEN
        self.open_list.put((self.k[x], x))
    

    def process_state(self, start):
        """
        Compute one iteration of D*

        start is the start state (note the goal state is fixed)
        """
        k_old, x = self.min_state()
        # print(k_old, x)
        if x == None:
            Exception('Priority queue is empty')
        # print(x.x, x.y)

        self.t[x] = Dstar.CLOSED

        # LOWER state
        if k_old < self.h[x]:
            # print('LOWER')
            for y in self.graph[x]:
                if self.h[y] <= k_old and self.h[x] > self.h[y] + x.cost_to(y):
                    self.b[x] = y
                    self.h[x] = self.h[y] + x.cost_to(y)

        # RAISE state
        elif k_old == self.h[x]:
            # print('RAISE')
            for y in self.graph[x]:
                # print(y in self.b)
                # print(y in self.h)
                # print(self.t[y] == Dstar.NEW)
                # print()
                if self.t[y] == Dstar.NEW \
                   or (self.b[y] == x and self.h[y] != self.h[x] + x.cost_to(y)) \
                   or (self.b[y] != x and self.h[y] > self.h[x] + x.cost_to(y)):
                    self.b[y] = x
                    self.insert(y, self.h[x] + x.cost_to(y))

        # "DIJKSTRA" state (effectively a dijkstra implementation)
        else:
            # print('DIJKSTRA')
            for y in self.graph[x]:
                # print('Considering ', f'({y.x}, {y.y})')
                if self.t[y] == Dstar.NEW \
                   or (self.b[y] == x and self.h[y] != self.h[x] + x.cost_to(y)):
                    self.b[y] = x
                    self.insert(y, self.h[x] + x.cost_to(y))
                else:
                    if self.b[y] != x and self.h[y] > self.h[x] + x.cost_to(y):
                        self.insert(x, self.h[x])
                    else:
                        if self.b[y] != x \
                           and self.h[x] > self.h[y] + y.cost_to(x) \
                           and self.t[y] == Dstar.CLOSED \
                           and self.h[y] > k_old:
                            self.insert(y, self.h[y])
            
        # input()
        return x # change here as I dont know why returning k_min would help
    

    def modify_cost(self, x, y, cval):
        x.make_obstacle()
        assert(x.cost_to(y) == cval)
        if self.t[x] == Dstar.CLOSED:
            print('ADDED')
            self.insert(x, self.h[x])
        return None # should be k_min idk how get that tho
    

    def plan(self, start):
        curr = None
        while curr != start:
            curr = self.process_state(start)

        self.path = []
        node = start
        while True:
            self.path.append(node)
            if node == self.goal:
                break
            node = self.b[node]

        return self.path
