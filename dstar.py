from lib.visualization import *
from lib.node import *
from lib.planner import *
from lib.environment import *
import numpy as np
import matplotlib.pyplot as plt

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

planner = Dstar(env.goal, env.graph)
path = planner.plan(env.start)

viz.plot_path(BasicNode.nodes_to_xy(path))
viz.show(msg='Showing path. Press enter to add point obstacle') 

x = np.random.choice(path)
y = planner.b[x]
planner.modify_cost(x,y,float('inf'))

path = planner.plan(env.start)
print(len(path))
viz.plot_path(BasicNode.nodes_to_xy(path), col='Purple')
viz.show(msg='Showing path. Press enter to quit')
