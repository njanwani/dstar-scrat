import matplotlib.pyplot as plt
import numpy as np

x = np.arange(-5, 5)
y = x.reshape(-1, 1)
print(x)
h = np.sin(x) * y**2

cs = plt.contourf(h, levels=np.linspace(-1,10,num=100), extend='both')
# cs.cmap.set_over('red')
# cs.cmap.set_under('blue')
# cs.changed()
plt.show()