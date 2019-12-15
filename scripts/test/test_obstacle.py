import argparse
import sys
import os
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
except IndexError:
    print("Cannot add the common path {}".format(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from arguments import add_arguments
from ilqr.obstacles import Obstacle
import numpy as np

argparser = argparse.ArgumentParser(description='CARLA CILQR')
add_arguments(argparser)
args = argparser.parse_args()
ob = Obstacle(args, 0, np.array([1.0, 2.0]))

npc_traj = np.random.random_sample((4,10))
l, q = ob.get_obstacle_cost_derivatives(npc_traj, 2, np.array([0.1, 2.0, 0.1, 0.1]))

print(l)
