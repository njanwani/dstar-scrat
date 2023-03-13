import matplotlib.pyplot as plt
import numpy as np

class Visualization:
    '''
    Plotter class for planning algorithms
    '''

    def __init__(self, xlim, ylim, ax):
        '''
        xlim:   (xmin, xmax) axis limits
        ylim:   (ymin, ymax) axis limits
        ax:     axes object
        '''

        self.xlim = xlim
        self.ylim = ylim
        self.ax = ax

 
    def show(self, msg='', wait=True):
        '''
        Shows the current ax. If wait is true, it will wait for a keyboard input
        '''

        plt.pause(0.001)
        if wait: return input(msg)
        
    
    def set_limits(self):
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)

    
    def plot_axes(self):
        '''
        Plots an empty pair of axes. Requires that a plt.axes object is passed in
        '''

        self.ax.scatter([], [])
        self.set_limits()

    
    def plot_nodes(self, pts, col='Blue', size=6):
        '''
        Plots nodes for planning algo.

        pts:    2D array of x,y positions --> [[x0, y0], [x1, y1], ...]
        '''
        self.ax.scatter(np.array(pts)[:, 0], np.array(pts)[:, 1], c=col, s=size)
        self.set_limits()


    def plot_graph(self, graph, col='Green', size=0.1):
        for key in graph:
            for node in graph[key]:
                self.ax.plot((key.x, node.x), (key.y, node.y), c=col, linewidth=size)


    def plot_path(self, path, col='Red', size=2):
        '''
        Plots path for planning algo.

        path:    2D array of x,y positions --> [[x0, y0], [x1, y1], ...]
        '''
        self.ax.plot(np.array(path)[:, 0], np.array(path)[:, 1], c=col, linewidth=size)