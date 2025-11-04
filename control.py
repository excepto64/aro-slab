#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin

from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 275.               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    
    t_idx = (int) (tcurrent / dt)

    print(t_idx)

    q_desired = trajs[t_idx][0][:]
    v_desired = trajs[t_idx][1][:]      # your array second dimension is actually velocity
    # You need a_desired too - compute numerical derivative:
    v_next = trajs[t_idx+1][1][:]
    a_desired = (v_next - v_desired) / dt
    
    # PD correction (optional but recommended)
    pos_err = q_desired - q
    vel_err = v_desired - vq

    a = a_desired + Kp*pos_err + Kv*vel_err

    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    sim.step(tau)
    
    # # From trajs get torques
    # q_current = trajs[(int) (tcur/dt)][0][:]
    # vq_current = trajs[(int) (tcur/dt)][1][:]

    # torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    # sim.step(torques)

def controllawBezier(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    
    t_idx = (int) (tcurrent)


    q_desired = trajs[t_idx][0].eval_horner(DT)
    v_desired = trajs[t_idx][1].eval_horner(DT)
    a_desired = trajs[t_idx][2].eval_horner(DT)
    
    print(q_desired, "\t", v_desired, "\t", a_desired)

    # PD correction (optional but recommended)
    pos_err = q_desired - q
    vel_err = v_desired - vq

    a = a_desired + Kp*pos_err + Kv*vel_err

    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    q_current = pin.integrate(robot.model, q_desired, tau*DT)

    data = robot.data.copy()  # Fresh data instance
    pin.forwardKinematics(robot.model, data, q_current)
    pin.updateFramePlacements(robot.model, data)  # Important!

    sim.step(tau)
    
    
if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    from trajectory import getTrajectory1, getTrajBezier
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    #setting initial configuration
    sim.setqsim(q0)
    
    
    #TODO this is just an example, you are free to do as you please.
    #In any case this trajectory does not follow the path 
    #0 init and end velocities
    def maketraj(q0,q1,T): #TODO compute a real trajectory !
        q_of_t = Bezier([q0,q0,q1,q1],t_max=T)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t
    
    P, I, D = Kp, 1, Kv
    
    
    #TODO this is just a random trajectory, you need to do this yourself
    
    max_time = 10
    num_of_steps = 10
    
    # trajs, time_steps = getTrajectory1(robot, cube, path, max_time, P, I, D)
    trajs, time_steps = getTrajBezier(robot, cube, path, max_time, num_of_steps)
    
    total_time = (int) (time_steps[-1])

    tcur = 0.

    dt = max_time/(len(path)-2)

    dt = max_time / num_of_steps

    print("traj",len(trajs))
    
    
    while tcur < total_time:
        print("tcur:", tcur, "\tt_total:", total_time)
        # print("control loop iteration:", (int) (tcur/dt), "of", (int) (total_time/dt))

        # rununtil(controllaw, dt, sim, robot, trajs, tcur, cube)
        # tcur += dt

        rununtil(controllawBezier, DT*1000, sim, robot, trajs, tcur, cube)
        tcur += dt
    
    
    