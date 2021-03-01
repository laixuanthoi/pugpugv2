from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from kinematic import Kinematic
from threading import Thread, Lock


fig = plt.figure()


def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax


def init():
    return lnBody, lf_ln, rf_ln, lb_ln, rb_ln, lf_pStart, rf_pStart, lb_pStart, rb_pStart, lf_pEnd, rf_pEnd, lb_pEnd, rb_pEnd,


ROLL = 0.0
PITCH = 0.0
YAW = 0.0

LegPoint = [[20, -100, 50, 1],   [100, -100, -50, 1],
            [-100, -100, 50, 1],  [-100, -100, -50, 1]]


def update(i):
    global ROLL, PITCH, YAW
    Lp = np.array(LegPoint)

    (roll, pitch, yaw) = (ROLL, PITCH, YAW)

    # center
    (x, y, z) = (0, 0, 0)
    (Tlf, Trf, Tlb, Trb) = Kinematic().bodyIK(
        roll, pitch, yaw, x, y, z)  # four jacobian

    FP = [0, 0, 0, 1]
    Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    CP = [x.dot(FP) for x in [Tlf, Trf, Tlb, Trb]]
    CPs = [CP[x] for x in [0, 1, 3, 2, 0]]

    # draw body
    lnBody.set_data([x[0] for x in CPs], [x[2] for x in CPs])
    lnBody.set_3d_properties([x[1] for x in CPs])

    # draw leg pair forward
    plf = [Tlf.dot(x) for x in Kinematic().calcLegPoints(
        Kinematic().legIK(np.linalg.inv(Tlf).dot(Lp[0])))]
    lf_ln.set_data([x[0] for x in plf], [x[2] for x in plf])
    lf_ln.set_3d_properties([x[1] for x in plf])

    lf_pStart.set_data([plf[0][0]], [plf[0][2]])
    lf_pStart.set_3d_properties([plf[0][1]])

    lf_pEnd.set_data([plf[4][0]], [plf[4][2]])
    lf_pEnd.set_3d_properties([plf[4][1]])

    # right
    prf = [Trf.dot(Ix.dot(x)) for x in Kinematic().calcLegPoints(
        Kinematic().legIK(Ix.dot(np.linalg.inv(Trf).dot(Lp[1]))))]
    rf_ln.set_data([x[0] for x in prf], [x[2] for x in prf])
    rf_ln.set_3d_properties([x[1] for x in prf])

    rf_pStart.set_data([prf[0][0]], [prf[0][2]])
    rf_pStart.set_3d_properties([prf[0][1]])

    rf_pEnd.set_data([prf[4][0]], [prf[4][2]])
    rf_pEnd.set_3d_properties([prf[4][1]])

    # behind

    # left
    plb = [Tlb.dot(x) for x in Kinematic().calcLegPoints(
        Kinematic().legIK(np.linalg.inv(Tlb).dot(Lp[2])))]
    lb_ln.set_data([x[0] for x in plb], [x[2] for x in plb])
    lb_ln.set_3d_properties([x[1] for x in plb])

    lb_pStart.set_data([plb[0][0]], [plb[0][2]])
    lb_pStart.set_3d_properties([plb[0][1]])

    lb_pEnd.set_data([plb[4][0]], [plb[4][2]])
    lb_pEnd.set_3d_properties([plb[4][1]])

    # right
    prb = [Trb.dot(Ix.dot(x)) for x in Kinematic().calcLegPoints(
        Kinematic().legIK(Ix.dot(np.linalg.inv(Trb).dot(Lp[3]))))]
    rb_ln.set_data([x[0] for x in prb], [x[2] for x in prb])
    rb_ln.set_3d_properties([x[1] for x in prb])

    rb_pStart.set_data([prb[0][0]], [prb[0][2]])
    rb_pStart.set_3d_properties([prb[0][1]])

    rb_pEnd.set_data([prb[4][0]], [prb[4][2]])
    rb_pEnd.set_3d_properties([prb[4][1]])

    return lnBody, lf_ln, rf_ln, lb_ln, rb_ln, lf_pStart, rf_pStart, lb_pStart, rb_pStart, lf_pEnd, rf_pEnd, lb_pEnd, rb_pEnd,


ax = setupView(200)
ax.view_init(elev=12., azim=28)

lnBody, = plt.plot([], [], [], 'bo-', lw=2, animated=True)  # body line
# left forward
lf_ln, = plt.plot([], [], [], 'k-', lw=3, animated=True)
lf_pStart, = plt.plot([], [], [], 'bo', lw=2, animated=True)
lf_pEnd, = plt.plot([], [], [], 'ro', lw=2, animated=True)

# right forward
rf_ln, = plt.plot([], [], [], 'k-', lw=3, animated=True)
rf_pStart, = plt.plot([], [], [], 'bo', lw=2, animated=True)
rf_pEnd, = plt.plot([], [], [], 'ro', lw=2, animated=True)

# left behind
lb_ln, = plt.plot([], [], [], 'k-', lw=3, animated=True)
lb_pStart, = plt.plot([], [], [], 'bo', lw=2, animated=True)
lb_pEnd, = plt.plot([], [], [], 'ro', lw=2, animated=True)

# right behind
rb_ln, = plt.plot([], [], [], 'k-', lw=3, animated=True)
rb_pStart, = plt.plot([], [], [], 'bo', lw=2, animated=True)
rb_pEnd, = plt.plot([], [], [], 'ro', lw=2, animated=True)

leg_animated = animation.FuncAnimation(
    fig, update, init_func=init, interval=1, blit=True)
