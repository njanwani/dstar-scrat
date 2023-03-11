from visualization import *
from planner import *
from node import *

# plot empty axes
fig, ax = plt.subplots()
viz = Visualization((-5, 5), (-5, 5), ax)
viz.plot_axes()
viz.show()

# plot points
pts = np.random.random((500,2)) * 10 - 5
pts = np.append(pts, (np.array([-4,-4]), np.array([4,4])), axis=0)
viz.plot_nodes(pts)
ax.set_title('bruh')
viz.show()

# generate graph and plot astar path
nodes = []
start, goal = None, None
for pt in pts:
    nodes.append(BasicNode(pt[0], pt[1]))
    if pt[0] == -4 and pt[1] == -4:
        start = nodes[-1]
    elif pt[0] == 4 and pt[1] == 4:
        goal = nodes[-1]

graph = {}
for v1 in nodes:
    graph[v1] = []
    for v2 in nodes:
        if v1.cost_to(v2) < 1 and v1 != v2:
            graph[v1].append(v2)

planner = Astar()
path = planner.plan(graph, start, goal)

pointpath = []

for node in path:
    pointpath.append([node.x, node.y])

viz.plot_path(pointpath)
viz.show() 