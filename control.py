#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin
from config import LEFT_HAND, RIGHT_HAND, LEFT_HOOK, RIGHT_HOOK, DT
from tools import getcubeplacement

# Constants:
KpHP = 18000
KdHP = 2.0 * np.sqrt(KpHP)
grasp_forceHP = 1800

gains_high = [KpHP, KdHP, grasp_forceHP]


# Constants:
Kp = 300
Kd = 2.0 * np.sqrt(Kp)
grasp_force = 50.

gains_normal = [Kp, Kd, grasp_force]

def controllaw(sim, robot, trajs, tcurrent, tmax, cube, gains, dt = DT):
    global int_err 
    q, vq = sim.getpybulletstate()

    pin.updateFramePlacements(robot.model, robot.data)
    pin.framesForwardKinematics(robot.model, robot.data, q)

    # Get desired q, v, a
    q_desired = trajs(tcurrent)
    v_desired = trajs.derivative(1)(tcurrent)
    a_desired = trajs.derivative(2)(tcurrent)
    
    # Calulcate PID error
    q_err = q_desired - q
    v_err = v_desired - vq
    
    Kp = gains[0]
    Kd = gains[1]
    force = gains[2]

    a = a_desired + (Kp * q_err) + (Kd * v_err)

    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    # Add "grasp force" so the cube does not get dropped
    # For left hand
    idx_L = robot.model.getFrameId(LEFT_HAND)
    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, idx_L, pin.LOCAL_WORLD_ALIGNED)
    f_left = np.array([0., -1., 0., 0., 0., 0.]) * force

    # For right hand
    idx_R = robot.model.getFrameId(RIGHT_HAND)
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, idx_R, pin.LOCAL_WORLD_ALIGNED)
    f_right = np.array([0., 1., 0., 0., 0., 0.]) * force

    tau_grasp = J_R.T @ f_right + J_L.T @ f_left

    # # Add grasp force to overall tau and step
    tau += tau_grasp 

    sim.step(tau)
    

    

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil, setupwithmeshcat
    import time
    from setup_meshcat import updatevisuals
    
    robot, sim, cube, viz = setupwithpybulletandmeshcat(url="tcp://127.0.0.1:6000")

    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath
    from trajectory import getTrajBezier
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    #path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    #np.savetxt('path_long.txt', path)
    path = np.loadtxt('path.txt')

    #displaypath(robot, cube, path, 0.001, viz)

    #setting initial configuration
    sim.setqsim(q0)
    sim.setTorqueControlMode()

    max_time = 1
    num_of_steps = 1000
    
    trajs, time_steps = getTrajBezier(robot, cube, path, max_time, num_of_steps)

    tcur = 0.
    dt = max_time/(len(time_steps))
    
    int_err = np.zeros_like(trajs(0))

    while tcur < max_time:
        controllaw(sim, robot, trajs, tcur, max_time, cube, gains_high, dt)

        tcur += dt
        time.sleep(0.01)
    time.sleep(3)
    