from lib.visualization import *
from lib.node import *
from lib.planner import *
from lib.environment import *
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(234)
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

planner = DstarFocused(env.goal, env.start, env.graph)
path = planner.plan(env.start)

viz.plot_path(BasicNode.nodes_to_xy(path))
for node in path:
    print(str(node))
viz.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck and processed nodes before obstacles and replanning: %s" % (planner.getCounts(), ))

# making obstacles at 25%, 50%, and 75% of the current path...
idx = len(path) // 4
x = path[idx]
r = path[idx - 1]
print(x, 'is now an obstacle')
y = planner.b[x]
planner.modify_cost(x,y,float('inf'), r)

idx = len(path) // 2
x = path[idx]
r2 = path[idx - 1]
print(x, 'is now an obstacle')
y = planner.b[x]
planner.modify_cost(x,y,float('inf'), r)

idx = 3 * len(path) // 4
x = path[idx]
r3 = path[idx - 1]
print(x, 'is now an obstacle')
y = planner.b[x]
planner.modify_cost(x,y,float('inf'), r)

# replanning
path = planner.replan(r)
viz.plot_path(BasicNode.nodes_to_xy(path), col='Purple')
viz.show(msg='Showing altered path. Press enter to quit')
print(f"associated onDeck and processed nodes: %s" % (planner.getCounts(), ))
