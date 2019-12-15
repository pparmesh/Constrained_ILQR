import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from sklearn.preprocessing import normalize
import math

class PolyRect:

    def __init__(self, dim):
        self.dimension = dim

        self.corners = np.zeros((4,2))
        self.axes = np.zeros((2,2))

        self.R = np.zeros((2,2))

    # Finds the corners and axes 
    def createCuboid(self, state):
        self.origin = state[0:2]
        self.orientation = state[2]

        # Creates Rotation Matrix for cuboid 
        self.R = np.array([[math.cos(self.orientation), -math.sin(self.orientation)],
                           [math.sin(self.orientation),  math.cos(self.orientation)]])

        self.findCorners()
        
    def findCorners(self):
        # Corners of a cuboid of length one and orientation of zero along x,y and z
        direction = np.array([[ 0.5, 0.5],[-0.5, 0.5], \
                              [ -0.5,-0.5],[0.5,-0.5]])

        # Dimension along x,y and z
        D = np.tile(self.dimension, (4, 1))

        # Cuboid scaled according to dimensions
        direction = direction*D

        # Origin of the cuboid
        O = np.tile(self.origin, (4,1))

        # Corners after rotation by R and translation by O
        self.corners = np.matmul(self.R, (direction).T).T + O
    
    def getPatch(self, ax, color='r'):
        # plt.close()
        # self.corners[[2,3]] = self.corners[[3,2]]
        rect = matplotlib.patches.Polygon(self.corners[:,:], color=color, alpha=0.50)

        # angle = np.pi*self.orientation/180.0
        # t2 = matplotlib.transforms.Affine2D().rotate_deg(angle) + ax.transData
        # rect.set_transform(t2)

        return rect
    
    def getCorners(self):
        return self.corners
