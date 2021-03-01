from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax


class Simulator():
    def __init__(self, pug):
        self.fig = plt.figure()
        self.pug = pug

    def update(self):
        roll, pitch, yaw = self.pug.state.roll, self.pug.state.pitch, self.pug.state.yaw
        Lp = self.pug.state.foot_locations
        (x, y, z) = (0, 0, 0)
