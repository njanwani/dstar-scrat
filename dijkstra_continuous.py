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
viz.show(msg='Showing axes. Press enter to plan a path')

# make environment
env = Uniform(np.random.random((1000,2)) * 10 - 5, mincost=0.5, nodetype=TopologyNode)
env.add_startgoal(start=[-4, -4], goal=[4, 4])

for v in env.graph.keys():
    v.set_cost(np.sin(v.x) + v.y**2)
# viz.plot_nodes(env.pts)
ax.set_title('Dijkstra path planning on contour $\sin(x) + y^2$') 

x = np.arange(-7, 7)
y = x.reshape(-1, 1)
h = np.sin(x) + y**2

cs = plt.contourf(x,y.flatten(),h, levels=np.linspace(-1,10,num=100), extend='both')
# plan initial path
planner = Astar(cost_multiplier=0)
path, total_cost = planner.plan(env.graph, env.start, env.goal)
viz.plot_path(BasicNode.nodes_to_xy(path), col='Black')
viz.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))


# for v in env.graph.keys():
#     v.set_cost(np.cos(v.y) + v.x**2)

# x = np.arange(-7, 7)
# y = x.reshape(-1, 1)
# h = np.cos(y) + x**2

# fig, ax = plt.subplots()
# viz2 = Visualization((-5, 5), (-5, 5), ax)
# ax.set_title('D* path planning on contour $x^2 + \sin(y)$') 
# viz2.plot_axes()

# cs = plt.contourf(x,y.flatten(),h, levels=np.linspace(-1,10,num=100), extend='both')
# # plan initial path
# planner = Astar(cost_multiplier=0)
# newpath, total_cost = planner.plan(env.graph, env.start, env.goal)
# viz2.plot_path(BasicNode.nodes_to_xy(newpath), col='Black')
# viz2.show(msg='Showing path. Press enter to add point obstacle') 
# print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))

# plot empty axes
fig2, ax2 = plt.subplots()
viz2 = Visualization((-5, 5), (-5, 5), ax2)
viz2.plot_axes()
viz2.show(msg='Showing axes. Press enter to plan a path')

np.random.seed(234) # choose a seed...
# make environment
env2 = Uniform(np.random.random((1000,2)) * 10 - 5, mincost=0.5, nodetype=TopologyNode)
env2.add_startgoal(start=[path[len(path) // 2].x, path[len(path) // 2].y], goal=[4, 4])

for v in env2.graph.keys():
    v.set_cost(np.sin(v.y) + v.x**2)
# viz.plot_nodes(env.pts)
ax2.set_title('Dijkstra path planning on contour $\sin(x) + y^2$') 

x = np.arange(-7, 7)
y = x.reshape(-1, 1)
h = np.sin(y) + x**2

cs = plt.contourf(x,y.flatten(),h, levels=np.linspace(-1,10,num=100), extend='both')
# plan initial path
planner2 = Astar(cost_multiplier=0)
newpath2, total_cost2 = planner2.plan(env2.graph, env2.start, env2.goal)
viz2.plot_path(BasicNode.nodes_to_xy(path[:len(path) // 2] + newpath2), col='Black')
viz2.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner2.getCounts(), ))


