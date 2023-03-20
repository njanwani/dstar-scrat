from lib.visualization import *
from lib.node import *
from lib.planner import *
from lib.environment import *
import numpy as np
import matplotlib.pyplot as plt

def replan(new_start, points, new_obstacles, cost_multiplier=1, prevCounts = (0, 0, 0)):
    env = OrderedList(points, new_obstacles)
    env.add_startgoal(start=new_start, goal=[4.0, 3.0])
    planner = Astar(cost_multiplier, prevCounts[0], prevCounts[1], prevCounts[2])
    return (planner.plan(env.graph, env.start, env.goal), planner.getCounts())

def path_cost(path):
    total_cost = 0
    for i in range(len(path) - 1):
        total_cost += path[i].cost_to(path[i+1])

    return total_cost

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
final_path = np.copy(path)

viz.plot_path(BasicNode.nodes_to_xy(path))
viz.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))
print(f"Total cost of path: %s" % (path_cost(final_path), ))


# making obstacle at 50% of the current path...
first_obstacles = []
idx = len(path) // 2
x = path[idx]
first_obstacles.append([x.x, x.y])
# print(x, 'is now an obstacle')

# replanning
new_start = [path[idx - 1].x, path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, first_obstacles, 1, planner.getCounts())
prev_final_idx = idx - 1
final_path = np.concatenate((final_path[0:prev_final_idx], new_path))
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Purple')
viz.plot_nodes(first_obstacles, col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck, processed nodes, and iterations: %s" % (new_counts, ))
print(f"Total cost of path: %s" % (path_cost(final_path), ))

# making obstacle at 25% of the new path...
new_obstacles = []
idx = len(path) // 4
x = new_path[idx]
new_obstacles.append([x.x, x.y])
# print(x, 'is now an obstacle')

# replanning
new_start = [new_path[idx - 1].x, new_path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, new_obstacles, 1, new_counts)
prev_final_idx += idx - 1
final_path = np.concatenate((final_path[0:prev_final_idx], new_path))
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Blue')
viz.plot_nodes(new_obstacles, col='Black', size=24)
viz.plot_nodes(first_obstacles, col='White', size=24)
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck, processed nodes, and iterations: %s" % (new_counts, ))
print(f"Total cost of path: %s" % (path_cost(final_path), ))

# D STAR
np.random.seed(234)
# plot empty axes
fig, ax = plt.subplots()
viz = Visualization((-5, 5), (-5, 5), ax)
viz.plot_axes()
viz.show(msg='Showing axes of D* Implementation. Press enter to continue')

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
obstacles = []
idx = len(path) // 2
old_pos = path[idx]
obstacles.append([old_pos.x, old_pos.y])
# print(x, 'is now an obstacle')
y = planner.b[old_pos]
planner.modify_cost(old_pos,y,float('inf'))

# replanning
path = planner.plan(env.start, y=path[idx-1])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Purple')
viz.plot_nodes(obstacles, col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck and processed nodes: %s" % (planner.getCounts(), ))

# making obstacle at 25% of the current path...
new_obstacles = []
idx = len(path) // 4
x = path[idx]
new_obstacles.append([x.x, x.y])
# print(x, 'is now an obstacle')
y = planner.b[x]
planner.modify_cost(x,y,float('inf'))

# reset old obstacle to free
y = planner.b[old_pos]
planner.modify_cost(old_pos, y, 0)

# replanning
path = planner.plan(env.start, y=path[idx-1])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Blue')
viz.plot_nodes(new_obstacles, col='Black', size=24)
viz.plot_nodes(obstacles, col='White', size=24)
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck and processed nodes: %s" % (planner.getCounts(), ))
