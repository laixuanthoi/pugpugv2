from pugpug.kinematic import Kinematic
from control.trotting import Trotting
from control.gait import Gait
from pugpug.state import State
import config as pug_cfg
import control.control_config as control_cfg
import numpy as np


class Body:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.Tlf, self.Trf, self.Tlb, self.Trb = 0.0, 0.0, 0.0, 0.0


class Leg:
    def __init__(self):
        self.foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))


class PugPug:
    def __init__(self):
        self.ik = Kinematic(pug_cfg.KinematicParams)
        self.body = Body()
        self.legs = Leg()
        # self.state = State()
        # self.trotting = Trotting(control_cfg.TrottingParams)
        # self.gait = Gait(control_cfg.GaitParams)

    def updateState(self):
        orientation = (self.body.roll, self.body.pitch, self.body.yaw)
        position = (self.body.x, self.body.y, self.body.z)
        (self.body.Tlf, self.body.Trf, self.body.Tlb,
         self.body.Trb) = self.ik.bodyIK(orientation, position)

    def runByCommand(self, cmd):
        pass
