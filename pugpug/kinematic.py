import numpy as np
from math import *


class Kinematic:
    def __init__(self, cfg):
        self.L = cfg.body_length
        self.W = cfg.body_width
        self.l1 = cfg.leg_length_1
        self.l2 = cfg.leg_length_2
        self.l3 = cfg.leg_length_3

    def bodyIK(self, orientation, position):  # calculate inverse kinematic of pugpug body
        omega, phi, psi = orientation
        xm, ym, zm = position
        Rx = np.array([[1, 0, 0, 0],
                       [0, np.cos(omega), -np.sin(omega), 0],
                       [0, np.sin(omega), np.cos(omega), 0], [0, 0, 0, 1]])
        Ry = np.array([[np.cos(phi), 0, np.sin(phi), 0],
                       [0, 1, 0, 0],
                       [-np.sin(phi), 0, np.cos(phi), 0], [0, 0, 0, 1]])
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0, 0],
                       [np.sin(psi), np.cos(psi), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Rxyz = Rx.dot(Ry.dot(Rz))

        T = np.array([[0, 0, 0, xm], [0, 0, 0, ym],
                      [0, 0, 0, zm], [0, 0, 0, 0]])
        Tm = T+Rxyz

        sHp = np.sin(np.pi/2)
        cHp = np.cos(np.pi/2)
        (L, W) = (self.L, self.W)

        return([Tm.dot(np.array([[cHp, 0, sHp, L/2], [0, 1, 0, 0], [-sHp, 0, cHp, W/2], [0, 0, 0, 1]])),
                Tm.dot(np.array(
                    [[cHp, 0, sHp, L/2], [0, 1, 0, 0], [-sHp, 0, cHp, -W/2], [0, 0, 0, 1]])),
                Tm.dot(np.array(
                    [[cHp, 0, sHp, -L/2], [0, 1, 0, 0], [-sHp, 0, cHp, W/2], [0, 0, 0, 1]])),
                Tm.dot(np.array([[cHp, 0, sHp, -L/2], [0, 1, 0, 0], [-sHp, 0, cHp, -W/2], [0, 0, 0, 1]]))])

    def ik_point_to_theta(self, point):  # calculate inverse kinematic of leg point
        x, y, z = point
        coxa, femur, tibia = self.l1, self.l2, self.l3
        c = atan2(abs(x), z)
        d = sqrt(x**2 + z**2) - coxa
        D = sqrt(y**2 + d**2)
        b = acos((tibia**2 + femur**2 - D**2) / (2*tibia*femur))
        # calculate kee angle
        a0 = atan2((sqrt(x**2 + z**2)-coxa), abs(y))
        a1 = acos((femur**2 + D**2 - tibia**2) / (2*femur*D))

        _hip = degrees(c)
        _knee = 180 - degrees(a0) - degrees(a1)
        _ankle = 180 - degrees(b)

        return _hip, _knee, _ankle

    # def ik_theta_to_point(self, thetas):
    #     (l1,l2,l3,l4)=(self.l1,self.l2,self.l3,self.l4)
    #     (theta1,theta2,theta3)=angles
    #     theta23=theta2+theta3

    #     T0=np.array([0,0,0,1])
    #     T1=T0+np.array([-l1*cos(theta1),l1*sin(theta1),0,0])
    #     T2=T1+np.array([-l2*sin(theta1),-l2*cos(theta1),0,0])
    #     T3=T2+np.array([-l3*sin(theta1)*cos(theta2),-l3*cos(theta1)*cos(theta2),l3*sin(theta2),0])
    #     T4=T3+np.array([-l4*sin(theta1)*cos(theta23),-l4*cos(theta1)*cos(theta23),l4*sin(theta23),0])

    #     return np.array([T0,T1,T2,T3,T4])
