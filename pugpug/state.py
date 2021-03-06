from enum import Enum
import numpy as np


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3


class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.height = -0.16
        self.activation = 0
        self.behavior_state = BehaviorState.REST
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))
