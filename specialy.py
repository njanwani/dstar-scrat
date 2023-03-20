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

top = np.vstack((np.linspace(-3, 3, num=10),[4]*10)).transpose()
bottom = np.vstack((np.linspace(-3, 3, num=10),[-4]*10)).transpose()
left = np.vstack(([-4]*10,np.linspace(-3, 3, num=10))).transpose()
right = np.vstack(([4]*10,np.linspace(-3, 3, num=10))).transpose()
# make environment
env = Uniform(np.append(np.append(np.append(left, top, axis=0), right, axis=0), bottom, axis=0), mincost=1.5)
env.add_startgoal(start=[-4, -4], goal=[4, 4])
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

# making obstacles at 25%, 50%, and 75% of the current path...
idx = len(path) // 2
x = path[idx]
print(x, 'is now an obstacle')
y = planner.b[x]
planner.modify_cost(x,y,float('inf'))

# idx = 3 * len(path) // 4
# x = path[idx]
# print(x, 'is now an obstacle')
# y = planner.b[x]
# planner.modify_cost(x,y,float('inf'))

# idx = len(path) // 4
# x = path[idx]
# print(x, 'is now an obstacle')
# y = planner.b[x]
# planner.modify_cost(x,y,float('inf'))

# replanning
path = planner.plan(env.start, y=path[idx-1])
viz.plot_path(BasicNode.nodes_to_xy(path), col='Purple')
viz.show(msg='Showing altered path. Press enter to quit')
