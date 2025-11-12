#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin
from config import LEFT_HAND, RIGHT_HAND, DT, EPSILON
from tools import getcubeplacement 


# Constants:
Kp = 300
Kd = 2.0 * np.sqrt(Kp)
grasp_force = 50.

cube_mass = 0.14
g = 9.81
v_inertia = 40

MAX_TIME = 2
NUM_OF_STEPS = 1000

GAINS = [Kp, Kd, grasp_force]

def controllaw(sim, robot, trajs, tcurrent, cube, gains = GAINS, dt = DT):
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
    graspforce = gains[2]
    
    a = a_desired + (Kp * q_err) + (Kd * v_err)

    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    # Add force to figth gravity
    f_g = 0.5 * cube_mass * g * v_inertia

    # Add "grasp force" so the cube does not get dropped
    # For left hand
    idx_L = robot.model.getFrameId(LEFT_HAND)
    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, idx_L, pin.LOCAL_WORLD_ALIGNED)
    f_left = np.array([0., -graspforce, f_g, 0., 0., 0.])

    # For right hand
    idx_R = robot.model.getFrameId(RIGHT_HAND)
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, idx_R, pin.LOCAL_WORLD_ALIGNED)
    f_right = np.array([0., graspforce, f_g, 0., 0., 0.])

    tau_grasp = J_R.T @ f_right + J_L.T @ f_left

    # Add grasp force to overall tau and step
    tau += tau_grasp

    sim.step(tau)


def runControlLaw(sim, robot, cube, path):
    '''
    For a path, get trajectories and runs the contorl law over these 
    trajectories. After follwoing the path, checks if it is within EPSILON of
    the goal state, if not, send current state to the replanner. 
    '''
    
    trajs, time_steps = getTrajBezier(robot, cube, path, MAX_TIME, NUM_OF_STEPS)
    print("Running Simulation... ")

    tcurrent = 0.
    dt = MAX_TIME/(len(time_steps))

    while tcurrent < MAX_TIME:
        controllaw(sim, robot, trajs, tcurrent, cube, dt=dt)
        # rununtil(controllaw, sim, robotsim, trajs, tcur, cubesim, dt)
        tcurrent += dt
        time.sleep(0.01)
    time.sleep(3)

    cubeq = getcubeplacement(cube)
    distToGoal = np.linalg.norm(cubeq.translation - CUBE_PLACEMENT_TARGET.translation)
    rotToGoal = np.linalg.norm(cubeq.rotation - CUBE_PLACEMENT_TARGET.rotation)
    
    if distToGoal + rotToGoal > EPSILON:
        print("Not at target location")
        replanner(sim, robot, cube, cubeq)
        return 
    else:
        print("Within tolerance of Target Location\n\n")
    return 




def replanner(sim, robot, cube, cubepos):
    '''
    Simple replanner to align cube with goal state if previous trajectory did
    not achieve goal state. This only works if the the cube is still in the 
    hands of the robot and there is a direct path (straight line) to the goal
    state.
    '''
    print("Replanning for cube position\n")

    q0, successinit = computeqgrasppose(robot, robot.q0, cube, cubepos, None)
    qe, successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    if successinit and successend:
        path = computeSimplepath(robot, cube, q0, qe, cubepos, CUBE_PLACEMENT_TARGET)
    else:
        print("No path available")
        return
    
    if len(path) != 200:
        return

    runControlLaw(sim, robot, cube, path)



if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil, setupwithmeshcat
    import time
    
    robot, sim, cube, viz = setupwithpybulletandmeshcat(url="tcp://127.0.0.1:6000")

    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath, computeSimplepath
    from trajectory import getTrajBezier
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    sim.setqsim(q0)
    sim.setTorqueControlMode()

    # COLLISION_THRESHOLD = min(collisionDistance(robot, q) for q in path)

    runControlLaw(sim, robot, cube, path)