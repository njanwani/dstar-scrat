import numpy as np
import bisect
from queue import PriorityQueue
from lib.node import *
    
class Planner:

    def __init__(self, init_onDeck = 0, init_processed = 0, init_iterations=0):
        self.nodes_onDeck = init_onDeck
        self.nodes_processed = init_processed
        self.iterations = init_iterations
    
    def plan():
        raise NotImplemented

    def getCounts(self):
        return (self.nodes_onDeck, self.nodes_processed, self.iterations)

    def addOnDeck(self):
        self.nodes_onDeck += 1

    def removeOnDeck(self):
        self.nodes_onDeck -= 1
    
    def addProcessed(self):
        self.nodes_processed += 1

    def addIterations(self):
        self.iterations += 1


class Astar(Planner):

    def __init__(self, cost_multiplier=1, init_onDeck = 0, init_processed = 0, init_iterations = 0):
        super().__init__(init_onDeck, init_processed, init_iterations)

        self.path = []
        self.cost_multiplier = cost_multiplier

    def plan(self, graph, start, goal):
        onDeck = []

        start.state = Node.ONDECK
        start.creach = 0.0
        start.cost = self.cost_multiplier * start.cost_to(goal)
        start.parent = None
        bisect.insort(onDeck, start)

        while True:
            # Grab the next node (first on the sorted on-deck list)
            node = onDeck.pop(0)

            # Check the neighbors
            for neighbor in graph[node]:
                self.addIterations()
                # Skip if already processed
                if neighbor.state == Node.DONE:
                    continue

                # Pre-compute the new cost to reach the neighbor
                cdelta = node.cost_to(neighbor)
                creach = node.creach + cdelta

                # If already on deck: skip if lower/same cost, else remove
                if neighbor.state == Node.ONDECK:
                    if neighbor.creach <= creach:
                        continue
                    onDeck.remove(neighbor)
                    self.removeOnDeck()

                # Add to the on-deck queue (in the right order)
                neighbor.state = Node.ONDECK
                self.addOnDeck()
                neighbor.creach = creach
                neighbor.cost = creach + self.cost_multiplier * neighbor.cost_to(goal)
                neighbor.parent = node
                bisect.insort(onDeck, neighbor)
            
            # Mark this node as processed
            node.state = Node.DONE

            # Check whether the goal is thereby done
            if goal.state == Node.DONE:
                break

            # Make sure we have something pending on the on-deck queue
            if not (len(onDeck) > 0):
                print("FAILED")
                return None, None
            
        # Create the path to the goal (backwards)
        node = goal
        path = []
        while node is not None:
            path.insert(0, node)

            # Move to the parent
            node = node.parent

        for node in graph.keys():
            if node.state == Node.DONE:
                self.addProcessed()
        # total_cost = 0
        # for i in range(len(path) - 1):
        #     total_cost += path[i].cost_to(path[i+1])
        total_cost = goal.creach

        return path, total_cost

        
                


    # OLD IMPLEMENTATION
    # def plan(self, graph, start, goal):
    #     """
    #     Classic (probably inefficient) Astar planner. Gets the job done.
    #     """

    #     pq = PriorityQueue()
    #     for node in graph[start]:
    #         pq.put((start.cost_to(node), [start, node]))
    #         node.state = Node.ONDECK
    #         self.addOnDeck()

    #     distances = {}

    #     while not pq.empty():
    #         dist, path = pq.get()
    #         path[-1].state = Node.DONE
    #         # self.addProcessed()

    #         if path[-1] not in distances:
    #             distances[path[-1]] = dist
    #         elif distances[path[-1]] > dist:
    #             distances[path[-1]] = dist
    #         elif distances[path[-1]] < dist:
    #             continue
                
    #         if path[-1] == goal:
    #             self.path = path
    #             for node in graph.keys():
    #                 if node.state == Node.DONE:
    #                     self.addProcessed()
    #             total_cost = 0
    #             for i in range(len(path) - 1):
    #                 total_cost += path[i].cost_to(path[i+1])

    #             return path, total_cost
            
    #         for node in graph[path[-1]]:
    #             if node not in path and node.state != Node.DONE:
    #                 if node.state != Node.ONDECK:
    #                     self.addOnDeck()
    #                 pq.put((dist + path[-1].cost_to(node) + self.cost_multiplier * node.cost_to(goal), path + [node]))
    #                 node.state = Node.ONDECK
                    

    #     print('FAILED')
    #     return None
    

class Dstar(Planner):
    NEW = 0
    OPEN = 1
    CLOSED = 2

    def __init__(self, g, graph, init_onDeck = 0, init_processed = 0,
                 init_iterations = 0):
        """
        Initialization function (many details are left out and can be seen in
        the D* paper) that declares variables and initializes dictionaries.
        
        g       is the goal
        graph   is the environment graph dictionary 
        """
        super().__init__(init_onDeck, init_processed, init_iterations)

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

        self.total_cost = 0

        # initialization steps
        for x in graph.keys():
            self.t[x] = Dstar.NEW

        self.goal = g
        self.h[g] = 0
        self.open_list.put((self.h[g], self.goal))
        self.t[g] = Dstar.OPEN
        self.b[self.goal] = None
        self.fullpath = []

    def min_state(self):
        """
        Returns the top of the OPEN priority queue
        """
        return self.open_list.get(block=False)
    

    def get_kmin(self):
        """
        Peeks at the top of the OPEN priority queue 
        """
        kmin, temp = self.open_list.get(block=False)
        self.open_list.put((kmin, temp))
        return kmin


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
        self.addOnDeck()
    

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

        if self.t[x] == Dstar.CLOSED:
            return 0
        self.t[x] = Dstar.CLOSED
        self.addProcessed()

        if start != None and self.t[start] == Dstar.CLOSED:
            return 0

        # RAISE state
        if k_old < self.h[x]:
            for y in self.graph[x]:
                self.addIterations()
                if self.h[y] <= k_old and self.h[x] > self.h[y] + x.cost_to(y):
                    self.b[x] = y
                    self.h[x] = self.h[y] + x.cost_to(y)

        # LOWER state
        if k_old == self.h[x]:
            for y in self.graph[x]:
                self.addIterations()
                if self.t[y] == Dstar.NEW \
                or (self.b[y] == x and self.h[y] != self.h[x] + x.cost_to(y)) \
                or (self.b[y] != x and self.h[y] > self.h[x] + x.cost_to(y)):
                    self.b[y] = x
                    self.insert(y, self.h[x] + x.cost_to(y))

        # "DIJKSTRA" state (effectively a dijkstra implementation)
        else:
            for y in self.graph[x]:
                self.addIterations()
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
            
        return self.get_kmin()
    

    def modify_cost(self, x, y, cval):
        if cval == float('inf'): x.make_obstacle()
        elif cval == 0: x.make_free()
        else:
            x.set_cost(cval)
        # assert(x.cost_to(y) == cval)
        if self.t[x] == Dstar.CLOSED:
            self.insert(x, self.h[x])
        return None #self.get_kmin()

    def plan(self, start, y=None):       
        curr_k = 0
        def get_condition(curr_k):
            if y == None:
                return self.t[start] != Dstar.CLOSED
            else:
                return curr_k < self.h[y]

        init = None
        if y == None:
            init = start 
       

        while get_condition(curr_k) and curr_k > -1:
            curr_k = self.process_state(init)

        self.path = []
        if y == None: 
            node = start
            self.total_cost = 0
        else: 
            node = y

        while True:
            self.path.append(node)
            if node == self.goal:
                break
            node = self.b[node]

        if y == None:
            self.fullpath = self.path
        else:
            self.fullpath = self.fullpath[:self.fullpath.index(y)] + self.path

        print('COST:', np.sum([self.fullpath[i].cost_to(self.fullpath[i + 1]) for i in range(len(self.fullpath) - 1)]))

        return self.path


class DstarFocused(Planner):
    NEW = 0
    OPEN = 1
    CLOSED = 2

    def __init__(self, goal, start, graph, init_onDeck = 0, init_processed = 0):
        """
        Initialization function (many details are left out and can be seen in
        the D* paper) that declares variables and initializes dictionaries.
        
        goal    is the goal
        start   is the start
        graph   is the environment graph dictionary 
        """
        super().__init__(init_onDeck, init_processed)

        self.t = {}                         # the types for a state X (i.e. NEW,
                                            # OPEN, or CLOSED)

        self.h = {}                         # the current ESTIMATES (i.e. robot
                                            # may find it is wrong) of the arc
                                            # costs from goal to X
        
        self.f = {}                         # the focused cost estimate, WITHOUT
                                            # adjustment for accrued bias.
                                            # Primarily used to break ties in
                                            # queue order
        
        self.fb = {}                        # the focused cost etimate, WITH
                                            # adjustment for accrued bias. This
                                            # is the primary cost by which the
                                            # queue is sorted

        self.k = {}                         # decides whether we are a RAISE, 
                                            # LOWER, or Dijkstra "mode". these
                                            # are the priorities used to order
                                            # the pqueue. This is the
                                            # simplification made by Gunters
                                            # approach
        
        self.r = {}                         # tracks the robot location where
                                            # a given node was put onto the
                                            # ONDECK
        
        self.b = {}                         # backpointers dictionary to later
                                            # reconstruct the path. Very
                                            # important for D*...

        self.open_list = PriorityQueue()    # OPEN list of states

        self.path = []                      # will eventually hold the path

        self.graph = graph                  # graph with neighbor relations

        self.r_curr = start                 # Stores the robots current
                                            # location in path execution

        self.r_prev = start                 # Stores the robots last location
                                            # where an obstacle was noticed

        self.d_bias = 0                     # Stores the accrued bias to
                                            # generate self.fb
        
        self.total_cost = 0

        # initialization steps
        for x in graph.keys():
            self.t[x] = Dstar.NEW
            self.r[x] = start

        self.goal = goal
        self.h[x] = 0
        self.f[x] = self.h[x] + self.goal.cost_to(start)
        self.fb[x] = self.f[x]
        self.k[x] = self.h[x]
        self.open_list.put(((self.fb[x], self.f[x], self.k[x]), self.goal))
        self.t[x] = Dstar.OPEN
        self.b[self.goal] = None

    def min_state(self):
        """
        Returns the top of the OPEN priority queue
        """
        (fb, f, k_old), x = self.open_list.get(block=False)
        while x != None:
            if self.r[x] != self.r_curr:
                #print("reinserting nodes! " + str(x.x) + "," + str(x.y))
                h_new = self.h[x]
                self.h[x] = self.k[x]
                self.insert(x, h_new)
            else:
                #print("returning node! " + str((x.x, x.y)) + "r[x]: " + str((self.r[x].x, self.r[x].y)))
                break
        return (k_old, x)
    

    def get_kmin(self, to_print = False):
        """
        Peeks at the top of the OPEN priority queue 
        """
        (fb, f, k_min), x = self.open_list.get(block=False)
        if to_print:
            print("fb: " + str(fb))
            print("f:" + str(f))
            print("k_min: " + str(k_min))
            print("x: " + str(x))
        self.open_list.put(((fb, f, k_min), x))
        return (f, k_min)


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

        if self.t[x] != Dstar.OPEN:
            self.addOnDeck()

        if self.r_curr != self.r_prev:
            print("existing bias cost: " + str(self.d_bias))
            self.d_bias += self.r_prev.cost_to(self.r_curr)
            print("updated bias cost: " + str(self.d_bias))
            print()
            self.r_prev = self.r_curr
        
        self.h[x] = h_new
        self.t[x] = Dstar.OPEN
        self.r[x] = self.r_curr
        self.f[x] = self.k[x] + x.cost_to(self.r_curr)
        self.fb[x] = self.f[x] + self.d_bias

        self.open_list.put(((self.fb[x], self.f[x], self.k[x]), x))
    

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

        #if self.t[x] == Dstar.CLOSED:
        #    return x

        self.t[x] = Dstar.CLOSED
        self.addProcessed()

        # LOWER state
        if k_old < self.h[x]:
            # print('LOWER')
            for y in self.graph[x]:
                if self.h[y] <= k_old and self.h[x] > self.h[y] + x.cost_to(y):
                    self.b[x] = y
                    self.h[x] = self.h[y] + x.cost_to(y)

        # RAISE state
        if k_old == self.h[x]:
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
        return self.get_kmin() # change here as I dont know why returning k_min would help
    

    def modify_cost(self, x, y, cval, r):
        self.r_curr = r
        x.make_obstacle()
        assert(x.cost_to(y) == cval)
        if self.t[x] == Dstar.CLOSED:
            print('ADDED')
            self.insert(x, self.h[x])
        return self.get_kmin()
    


    def plan(self, start):
        # curr = None
        # while curr != start:
        #     curr = self.process_state(start)

        val = None
        while self.t[start] != Dstar.CLOSED:
            val = self.process_state(start)
        

        self.path = []
        node = start
        while True:
            self.path.append(node)
            if node == self.goal:
                break
            node = self.b[node]

        total = 0
        for i in range(len(self.path) - 1):
            total += self.path[i].cost_to(self.path[i+1])
        
        self.total_cost += total
        
        print("COST: " + str(self.total_cost))
        return self.path

    def replan(self, start):

        val = (-1, -1)
        count = 0
        while val < (self.k[start], self.h[start]):#count < 3:#not self.open_list.empty(): #self.h[start]:
            if val[0] > 10000:
                count += 1
            val = self.process_state(start)
            #print(val[0])
        
        #print(val[0])

            
        self.path = []
        node = start
        while True:
            self.path.append(node)
            # val = (-1, -1)
            # while val[0] <=  10000: #self.h[start]:
            #     val = self.process_state(start)
            #     assert(type(val) != type(start))
            # print("val: " + str(val))
            if node == self.goal:
                break
            node = self.b[node]

        total = 0
        for i in range(len(self.path) - 1):
            total += self.path[i].cost_to(self.path[i+1])
        
        self.total_cost += total
        
        print("COST: " + str(self.total_cost))
        return self.path