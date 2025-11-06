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
Kp = 20               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    
    t_idx = (int) (tcurrent / dt)

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

    print("tau:", tau)

    sim.step(tau)
    
    

def controllawBezier(sim, robot, trajs, tcurrent, cube):
    q, v = sim.getpybulletstate()
    
    t_idx = (int) (tcurrent)
    print("t_idx", t_idx)

    b = trajs

    q_desired = b(tcurrent)
    v_desired = b.derivative(1)(tcurrent)
    a_desired = b.derivative(2)(tcurrent)
    

    # PD correction (optional but recommended)
    a_cmd = a_desired #+ Kp*(q_desired - q) + Kv*(v_desired - v)
    tau_des = pin.rnea(robot.model, robot.data, q_desired, v_desired, a_desired)
    tau_err = Kp * (q_desired - q) + Kv * (v_desired - v) 
    tau = tau_des + tau_err

    print("tau:\n", tau)

    sim.step(tau)

    return tau

    

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil, setupwithmeshcat
    from config import DT
    import time
    
    robotsim, sim, cubesim = setupwithpybullet()
    robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6003")
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath
    from trajectory import getTrajectory1, getTrajBezier
    
    q0,successinit = computeqgrasppose(robotsim, robotsim.q0, cubesim, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robotsim, robotsim.q0, cubesim, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(robotsim, cubesim, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    #setting initial configuration
    sim.setqsim(q0)
    
    
    P, I, D = Kp, 1, Kv    
    max_time = 10
    num_of_steps = 100
    
    # trajs, time_steps = getTrajectory1(robot, cube, path, max_time, P, I, D)
    trajs, time_steps = getTrajBezier(robot, cube, path, max_time, num_of_steps)
    
    total_time = (int) (time_steps[-1])

    tcur = 0.

    dt = max_time/(len(path)-2)

    dt = max_time / num_of_steps

    # print("traj",len(trajs))

    displaypath(robot, cube, path, 1/500, viz)
    
    
    while tcur < total_time:
    #     # print("tcur:", tcur, "\tt_total:", total_time)
    #     # print("control loop iteration:", (int) (tcur/dt), "of", (int) (total_time/dt))

    #     # rununtil(controllaw, dt, sim, robot, trajs, tcur, cube)
    #     # tcur += dt
        
        controllawBezier(sim, robotsim, trajs, tcur, cubesim)

    #     # rununtil(controllawBezier, DT*1000, sim, robot, trajs, tcur, cube)
        tcur += dt
        time.sleep(0.01)
    
    
    