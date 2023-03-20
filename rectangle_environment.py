from lib.visualization import *
from lib.node import *
from lib.planner import *
from lib.environment import *
import numpy as np
import matplotlib.pyplot as plt

def replan(new_start, points, new_obstacles, cost_multiplier=1, prevCounts = (0, 0)):
    env = OrderedList(points, new_obstacles)
    env.add_startgoal(start=new_start, goal=[4.0, 3.0])
    planner = Astar(cost_multiplier, prevCounts[0], prevCounts[1])
    return (planner.plan(env.graph, env.start, env.goal), planner.getCounts())

np.random.seed(234)
# plot empty axes
fig, ax = plt.subplots()
viz = Visualization((-5, 5), (-5, 5), ax)
viz.plot_axes()
viz.show(msg='Showing axes. Press enter to continue')

# make environment
horizontal = np.linspace(-4, 4, num=9)
vertical = np.linspace(-3, 3, num=7)
top_line = np.zeros((len(horizontal), 2))
top_line[:, 0] = horizontal
top_line[:, 1] = np.array([3.0]*9)
right_line = np.zeros((len(vertical), 2))
right_line[:, 0] = np.array([4.0]*7)
right_line[:, 1] = np.array(np.flip(vertical))
bot_line = np.zeros((len(horizontal), 2))
bot_line[:, 0] = np.flip(horizontal)
bot_line[:, 1] = np.array([-3.0]*9)
left_line = np.zeros((len(vertical), 2))
left_line[:, 0] = np.array([-4.0]*7)
left_line[:, 1] = np.array(vertical)
rectangle = np.concatenate((top_line, right_line[1:, :], bot_line[1:, :], left_line[1:-1, :]))
points = rectangle.tolist()
env = OrderedList(points)
env.add_startgoal(start=[-4.0, -3.0], goal=[4.0, 3.0])
viz.plot_nodes(env.pts)
ax.set_title('A* path planning algorithm on rectangle') 
viz.show(msg='Showing states. Press enter to continue')

# # plot graph connections
viz.plot_graph(env.graph)
viz.show(msg='Showing graph. Press enter to continue')
planner = Astar()
path, total_cost = planner.plan(env.graph, env.start, env.goal)

viz.plot_path(BasicNode.nodes_to_xy(path))
viz.show(msg='Showing path. Press enter to add point obstacle') 

# making obstacle at 50% of the current path...
new_obstacles = []
idx = len(path) // 2
x = path[idx]
new_obstacles.append([x.x, x.y])
print(x, 'is now an obstacle')

# replanning
new_start = [path[idx - 1].x, path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, new_obstacles, 1, planner.getCounts())
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Purple')
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck and processed nodes: %s" % (new_counts, ))

# making obstacle at 50% of the new path...
new_obstacles = []
idx = len(path) // 4
x = new_path[idx]
new_obstacles.append([x.x, x.y])
print(x, 'is now an obstacle')

# replanning
new_start = [new_path[idx - 1].x, new_path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, new_obstacles, 1, planner.getCounts())
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Blue')
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck and processed nodes: %s" % (new_counts, ))
print(f"associated onDeck and processed nodes before obstacles and replanning: %s" % (planner.getCounts(), ))
print(f"Total cost of original path: %s" % (total_cost, ))

# D STAR
np.random.seed(234)
# plot empty axes
fig, ax = plt.subplots()
viz = Visualization((-5, 5), (-5, 5), ax)
viz.plot_axes()
viz.show(msg='Showing axes. Press enter to continue')

# make environment
env = OrderedList(points)
env.add_startgoal(start=[-4.0, -3.0], goal=[4.0, 3.0])
viz.plot_nodes(env.pts)
ax.set_title('D* path planning algorithm') 
viz.show(msg='Showing states. Press enter to continue')

# plot graph connections
viz.plot_graph(env.graph)
viz.show(msg='Showing graph. Press enter to continue')

planner = Dstar(env.goal, env.graph)
path = planner.plan(env.start)

viz.plot_path(BasicNode.nodes_to_xy(path))
viz.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck and processed nodes before obstacles and replanning: %s" % (planner.getCounts(), ))

# making obstacle at 50% of the current path...
idx = len(path) // 2
x = path[idx]
print(x, 'is now an obstacle')
y = planner.b[x]
planner.modify_cost(x,y,float('inf'))

# replanning
path = planner.plan(path[idx - 1])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Purple')
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck and processed nodes: %s" % (planner.getCounts(), ))
