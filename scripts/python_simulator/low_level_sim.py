import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from sklearn.preprocessing import normalize
from matplotlib import animation
import math
from PolyRect import PolyRect
PI = math.pi


##
total_states = [np.array([10.0, -2.0, 0]),np.array([15.0, -2.0, PI/6]),np.array([20.0, -2.0, PI/3])]
fig = plt.figure(figsize=(25,5))
# plt.axis('equal')
ax = fig.add_subplot(111)
ax.axis('equal')
ax.set_xlim(0, 50)
ax.set_ylim(-50, 50)
ax.axhline(y=4, c='k', lw='4')
ax.axhline(y=0, c='k', lw='2', ls='--')
ax.axhline(y=-4, c='k', lw='4')

ego_dims = np.array([2, 1])
ego_init = np.array([0.0, 0.0, 0.0])

xdata, ydata = [], []
ln, = plt.plot([], [], 'go')
ln1, = plt.plot([], [], 'co')
line = [ln,ln1,]
lines = list(line)

ego_cube = PolyRect(ego_dims) # instantiate ego car
# ego_cube.createCuboid(total_states[0])
# print(ego_cube.getCorners())
ego_patch = ego_cube.getPatch(ax)

ax.add_patch(ego_patch)
#########################################
total_states1 = [np.array([35.0, 2.0, 0]),np.array([30.0, 2.0, PI/6]),np.array([25.0, 2.0, PI/3])]
ego_dims1 = np.array([2, 1])
ego_init1 = np.array([0.0, 0.0, 0.0])


ego_cube1 = PolyRect(ego_dims) # instantiate ego car
# ego_cube.createCuboid(total_states[0])
# print(ego_cube.getCorners())
ego_patch1 = ego_cube1.getPatch(ax)

ax.add_patch(ego_patch1)
pat = [ego_patch,ego_patch1] + lines

def init():
    return pat

def animate(i):
    # Get new corners of cuboid
    curr_state = total_states[i]
    ego_cube.createCuboid(curr_state)
    new_corner = ego_cube.getCorners()
    # new_corner =  np.array([[3+i, 0], [5+i, 0.], [5, 2+i], [3, 2+i]])
    ego_patch.set_xy(new_corner)
    # Set patch corners to new corners
    xdata = [15]
    ydata = [0]
    xdata1 = [20]
    ydata1 = [0]
    
    ln.set_data(xdata, ydata)
    ln1.set_data(xdata1, ydata1)

    curr_state1 = total_states1[i]
    ego_cube1.createCuboid(curr_state1)
    new_corner1 = ego_cube1.getCorners()
    # new_corner =  np.array([[3+i, 0], [5+i, 0.], [5, 2+i], [3, 2+i]])
    ego_patch1.set_xy(new_corner1)
    return pat

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(total_states),
                               interval=1000,
                               blit=True)
plt.show()
