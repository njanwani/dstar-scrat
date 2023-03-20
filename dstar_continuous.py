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
ax.set_title('D* path planning on contour $\sin(x) + y^2$') 

x = np.arange(-7, 7)
y = x.reshape(-1, 1)
h = np.sin(x) + y**2

cs = plt.contourf(x,y.flatten(),h, levels=np.linspace(-1,10,num=100), extend='both')
# plan initial path
planner = Dstar(env.goal, env.graph)
path = planner.plan(env.start)
viz.plot_path(BasicNode.nodes_to_xy(path), col='Black')
viz.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))


for v in env.graph.keys():
    planner.modify_cost(v, None, np.cos(v.y) + v.x**2)

x = np.arange(-7, 7)
y = x.reshape(-1, 1)
h = np.cos(y) + x**2

fig, ax = plt.subplots()
viz2 = Visualization((-5, 5), (-5, 5), ax)
ax.set_title('D* path planning on contour $x^2 + \sin(y)$') 
viz.plot_axes()

cs = plt.contourf(x,y.flatten(),h, levels=np.linspace(-1,10,num=100), extend='both')
# plan initial path
path = planner.plan(env.start, path[len(path) // 2])
viz2.plot_path(BasicNode.nodes_to_xy(planner.fullpath), col='Black')
ax.set_xlim((-5,5))
ax.set_ylim((-5,5))
viz2.show(msg='Showing path. Press enter to add point obstacle') 
print(f"associated onDeck, processed nodes, and iterations before obstacles and replanning: %s" % (planner.getCounts(), ))


