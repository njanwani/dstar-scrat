from lib.visualization import *
from lib.node import *
from lib.planner import *
from lib.environment import *
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(234) # choose a seed...

# plot empty axes
fig, ax = plt.subplots()
viz = Visualization((-5, 5), (-5, 5), ax)
viz.plot_axes()
viz.show(msg='Showing axes. Press enter to continue')

# make environment
env = Uniform(np.random.random((500,2)) * 10 - 5, mincost=1)
env.add_startgoal(start=[-4, -4], goal=[4, 4])
viz.plot_nodes(env.pts)
ax.set_title('D* path planning algorithm') 
viz.show(msg='Showing states. Press enter to continue')

# plot graph connections
viz.plot_graph(env.graph)
viz.show(msg='Showing graph. Press enter to continue')

# plan initial path
planner = Dstar(env.goal, env.graph)
path = planner.plan(env.start)
viz.plot_path(BasicNode.nodes_to_xy(path))
viz.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))

# making obstacles at 25%, 50%, and 75% of the current path...
obstacles = []

def make_obstacles(at=[0.25, 0.50, 0.75]):
    """
    Makes obstacles at the % in path specified by at. Returns the index of the 
    path before the first obstacle specified
    """
    global path, obstacles, planner
    for step in at:
        idx = int(step * len(path))
        x = path[idx]
        obstacles.append(x)
        y = planner.b[x]
        planner.modify_cost(x,y,float('inf'))
    return int(at[0] * len(path)) - 1

y = make_obstacles()

# replanning
path = planner.plan(env.start, y=path[y])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Purple')
viz.plot_nodes(BasicNode.nodes_to_xy(obstacles), col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to add newer obstacles')
print(f"associated onDeck, processed nodes, and iterations: %s" % (planner.getCounts(), ))

y = make_obstacles()

# replanning
path = planner.plan(env.start, y=path[y])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Blue')
viz.plot_nodes(BasicNode.nodes_to_xy(obstacles), col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to add newer obstacles')
print(f"associated onDeck, processed nodes, and iterations: %s" % (planner.getCounts(), ))

y = make_obstacles()

# replanning
path = planner.plan(env.start, y=path[y])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Violet')
viz.plot_nodes(BasicNode.nodes_to_xy(obstacles), col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to add newer obstacles')
print(f"associated onDeck, processed nodes, and iterations: %s" % (planner.getCounts(), ))

y = make_obstacles()

# replanning
path = planner.plan(env.start, y=path[y])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Indigo')
viz.plot_nodes(BasicNode.nodes_to_xy(obstacles), col='Black', size=24)
viz.show(msg='Showing altered path. Press enter to show final path')
print(f"associated onDeck, processed nodes, and iterations: %s" % (planner.getCounts(), ))


viz.plot_path(BasicNode.nodes_to_xy(planner.fullpath), col='Gold')
viz.show(msg='Showing full path. Press enter to quit')

