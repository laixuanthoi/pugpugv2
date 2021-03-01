from pugpug.kinematic import Kinematic
from control.trotting import Trotting
from control.gait import Gait
from pugpug.state import State
import config as pug_cfg
import control.control_config as control_cfg


class PugPug:
    def __init__(self):
        self.state = State()
        self.ik = Kinematic(pug_cfg.KinematicParams)
        self.trotting = Trotting(control_cfg.TrottingParams)
        self.gait = Gait(control_cfg.GaitParams)

    def runByCommand(self, cmd):
        pass
