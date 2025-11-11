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
Kp = 300
Kd = 2.0 * np.sqrt(Kp)
grasp_force = 50.

cube_mass = 0.14
g = 9.81
v_inertia = 40

gains_normal = [Kp, Kd, grasp_force]
COLLISION_THRESHOLD = 1e-3

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
    graspforce = gains[2]
    
    a = a_desired + (Kp * q_err) + (Kd * v_err)

    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    # Add force to figth gravity
    f_g = 0.5 * cube_mass * g * v_inertia


    # Add "grasp force" so the cube does not get dropped
    # For left hand
    idx_L = robot.model.getFrameId(LEFT_HAND)
    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, idx_L, pin.LOCAL_WORLD_ALIGNED)
    f_left = np.array([0., -grasp_force, f_g, 0., 0., 0.])

    # For right hand
    idx_R = robot.model.getFrameId(RIGHT_HAND)
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, idx_R, pin.LOCAL_WORLD_ALIGNED)
    f_right = np.array([0., grasp_force, f_g, 0., 0., 0.])

    tau_grasp = J_R.T @ f_right + J_L.T @ f_left

    # Add grasp force to overall tau and step
    tau += tau_grasp

    sim.step(tau)
    
    # Recursive loop to drag q back onto the path if it strays too far from 
    # obstacleAvoidanceTau(sim, robot, trajs, tcurrent, tmax, cube, dt)


def runControlLaw(sim, robot, cube, path, num_of_steps, max_time, gains):

    print("Running Simulation: ")
    
    trajs, time_steps = getTrajBezier(robot, cube, path, max_time, num_of_steps)

    tcurrent = 0.
    dt = max_time/(len(time_steps))

    while tcurrent < max_time:
        controllaw(sim, robot, trajs, tcurrent, max_time, cube, gains, dt)
        # rununtil(controllaw, sim, robotsim, trajs, tcur, cubesim, dt)
        tcurrent += dt
        time.sleep(0.01)
    time.sleep(3)


# Extensions:
def obstacleAvoidanceTau(sim, robot, trajs, tcurrent, tmax, cube, dt=DT):

    
    q = trajs(tcurrent)

    colDist = collisionDistance(robot, q)



    if colDist < COLLISION_THRESHOLD:
        print("Avoiding Obstacle")
        print(COLLISION_THRESHOLD)
        # getCollTau(robot, q)
        controllaw(sim, robot, trajs, tcurrent, tmax, cube, gains_high, dt)
    else:
        return
    


def getCollTau(robot, q, trajs, tcurrent):


    # expectedq = trajs(tcurrent)

    # return pin.rnea(robot.model, robot.data, expectedq, np.zeros_like(q), np.zeros_like(q))
    return

def collisionDistance(robot, q):
     '''Return the minimal distance between robot and environment. '''
     pin.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data, q)
     if pin.computeCollisions(robot.collision_model,robot.collision_data, False): 0
     idx = pin.computeDistances(robot.collision_model,robot.collision_data)
     return robot.collision_data.distanceResults[idx].min_distance




def replanner(sim, robot, cube, gains):
    q0, successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe, successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    if successinit and successend:
        path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    else:
        print("No path available")
        return
    max_time = 1
    num_of_steps = 1000

    runControlLaw(sim, robot, cube, path, num_of_steps, max_time, gains)
    return



    

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
    path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    #np.savetxt('path_long.txt', path)
    # path = np.loadtxt('path.txt')

    # displaypath(robot, cube, path, 0.01, viz)

    #setting initial configuration
    sim.setqsim(q0)
    sim.setTorqueControlMode()

    # COLLISION_THRESHOLD = min(collisionDistance(robot, q) for q in path)

    max_time = 1
    num_of_steps = 1000
    
    int_err = np.zeros_like(path[0])

    runControlLaw(sim, robot, cube, path, num_of_steps, max_time, gains_normal)
    