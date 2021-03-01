import numpy as np


class Trotting:
    def __init__(self, cfg):
        # step_length, step_height, step_width
        self.Sl, self.Sh, self.Sw = cfg.config_trotting()
        self.t0 = 100
        self.t1 = 150
        self.t2 = 50
        self.t3 = 100
        self.sumT = self.t0 + self.t1 + self.t2 + self.t3

        self.P0 = (-25, 250, 0)
        self.P1 = (self.P0[0] - self.Sl/2, self.P0[1], 0)
        self.P2 = (self.P1[0] + self.Sl, self.P1[1] - self.Sh, 0)
        self.P3 = (self.P2[0], self.P2[1] + self.Sh, 0)

    def calLegs(self, t):
        if t < 0:
            return self.P0
        elif t < self.t0:  # P0 -> P1 for t0
            tp = t/self.t0
            return (self.P0[0] - (self.Sl/2)*tp, self.P0[1], 0)
        elif t < self.t0 + self.t1:  # P1->P2 for t1
            td = t - self.t0
            tp = td / self.t1
            return (self.P1[0] + self.Sl*tp, self.P1[1] - self.Sh * tp, 0)

        elif t < self.t0 + self.t1 + self.t2:  # P2->P3 for t2
            td = t - (self.t0 + self.t1)
            tp = td / self.t2
            return (self.P2[0], self.P2[1] + self.Sh * tp, 0)

        elif t < self.t0 + self.t1 + self.t2 + self.t3:  # P3 ->P0 for t3
            td = t - (self.t0 + self.t1 + self.t2)
            tp = td/self.t3
            return (self.P3[0] - (self.Sl/2)*tp, self.P3[1], 0)
