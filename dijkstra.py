from lib.visualization import *
from lib.node import *
from lib.planner import *
from lib.environment import *
import numpy as np
import matplotlib.pyplot as plt

def generate_obstacles(path, old_obstacles=[]):
    # making obstacles at 25%, 50%, and 75% of the current path...
    obstacles = old_obstacles
    idx = len(path) // 2
    x = path[idx]
    obstacles.append([x.x, x.y])
    # print(x, 'is now an obstacle')

    idx = 3 * len(path) // 4
    x = path[idx]
    obstacles.append([x.x, x.y])
    # print(x, 'is now an obstacle')

    idx = len(path) // 4
    x = path[idx]
    obstacles.append([x.x, x.y])
    # print(x, 'is now an obstacle')

    return obstacles, idx

def replan(new_start, points, new_obstacles, cost_multiplier=1, prevCounts = (0, 0, 0)):
    points.remove(new_start)
    env = Uniform(points, new_obstacles)
    env.add_startgoal(start=new_start, goal=[4, 4])
    planner = Astar(cost_multiplier, prevCounts[0], prevCounts[1], prevCounts[2])
    return (planner.plan(env.graph, env.start, env.goal), planner.getCounts())

np.random.seed(234)
# plot empty axes
fig, ax = plt.subplots()
viz = Visualization((-5, 5), (-5, 5), ax)
viz.plot_axes()
viz.show(msg='Showing axes. Press enter to continue')

# make environment
points = np.random.random((500,2)) * 10 - 5
points = points.tolist()
env = Uniform(points)
env.add_startgoal(start=[-4, -4], goal=[4, 4])
viz.plot_nodes(env.pts)
ax.set_title('Dijkstra path planning algorithm') 
viz.show(msg='Showing states. Press enter to continue')

# plot graph connections
viz.plot_graph(env.graph)
viz.show(msg='Showing graph. Press enter to continue')

planner = Astar(0)
path, total_cost = planner.plan(env.graph, env.start, env.goal)

viz.plot_path(BasicNode.nodes_to_xy(path))
viz.show(msg='Showing path. Press enter to add point obstacle')
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))
print(f"Total cost of original path: %s" % (total_cost, ))

# replanning
obstacles, idx = generate_obstacles(path)
new_start = [path[idx - 1].x, path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, obstacles, 0, planner.getCounts())
total_cost += new_total_cost
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Purple')
viz.plot_nodes(obstacles, col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to add newer obstacles')
print(f"associated onDeck, processed nodes, and iterations: %s" % (new_counts, ))
print(f"Total cost of path: %s" % (total_cost, ))

# replanning
obstacles, idx = generate_obstacles(new_path)
new_start = [new_path[idx - 1].x, new_path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, obstacles, 0, new_counts)
total_cost += new_total_cost
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Blue')
viz.plot_nodes(obstacles, col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to add newer obstacles')
print(f"associated onDeck, processed nodes, and iterations: %s" % (new_counts, ))
print(f"Total cost of path: %s" % (total_cost, ))

# replanning
obstacles, idx = generate_obstacles(new_path)
new_start = [new_path[idx - 1].x, new_path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, obstacles, 0, new_counts)
total_cost += new_total_cost
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Violet')
viz.plot_nodes(obstacles, col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to add newer obstacles')
print(f"associated onDeck, processed nodes, and iterations: %s" % (new_counts, ))
print(f"Total cost of path: %s" % (total_cost, ))

# replanning
obstacles, idx = generate_obstacles(new_path)
new_start = [new_path[idx - 1].x, new_path[idx - 1].y]
(new_path, new_total_cost), new_counts = replan(new_start, points, obstacles, 0, new_counts)
total_cost += new_total_cost
viz.plot_path(BasicNode.nodes_to_xy(new_path), col='Indigo')
viz.plot_nodes(obstacles, col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck, processed nodes, and iterations: %s" % (new_counts, ))
print(f"Total cost of path: %s" % (total_cost, ))