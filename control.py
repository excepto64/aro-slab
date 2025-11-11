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
KiHP = 1.0 * KpHP
KdHP = 2 * np.sqrt(KpHP)
grasp_forceHP = 1800

gains_high = [KpHP, KiHP, KdHP, grasp_forceHP]

COLLISION_THRESHOLD = 1e-3


# Constants:
Kp = 300
Ki = 1.0 * Kp 
Kd = 2.0 * np.sqrt(Kp)
grasp_force = 300

gains_normal = [Kp, Ki, Kd, grasp_force]

INT_DECAY = 0.98

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
    
    # Decay of integral error
    int_err *= INT_DECAY
    int_err += q_err * dt 

    # Reduce gains when lowering the cube
    t_lowering = tmax * 0.9

    
    Kp_phase = gains[0]
    Ki_phase = gains[1]
    Kd_phase = gains[2]
    force_phase = gains[3]

    a = a_desired + (Kp_phase * q_err) + (Kd_phase*v_err) + (Ki_phase * int_err)

    # # Dampen hand joint accelerations
    # hand_damping = 10  # Reduce to 10% of commanded acceleration
    # a[8] *= hand_damping   # left hand
    # a[14] *= hand_damping  # right hand

    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    # Add "grasp force" so the cube does not get dropped
    # For left hand
    idx_L = robot.model.getFrameId(LEFT_HAND)
    oML = robot.data.oMf[idx_L]
    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    trans_err = oMcubeL.translation - oML.translation
    rot_err = pin.log3(oMcubeL.rotation @ oML.rotation.T)
    wrenchL = pin.Motion(np.hstack((trans_err * force_phase, rot_err)) )
    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, idx_L, pin.LOCAL_WORLD_ALIGNED)

    # For right hand
    idx_R = robot.model.getFrameId(RIGHT_HAND)
    oMR = robot.data.oMf[idx_R]
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    trans_err = oMcubeR.translation - oMR.translation
    rot_err = pin.log3(oMcubeR.rotation @ oMR.rotation.T)
    wrenchR = pin.Motion(np.hstack((trans_err * force_phase, rot_err)) )
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, idx_R, pin.LOCAL_WORLD_ALIGNED)

    cube_mass = 0.14  # kg, whatever your cube weighs
    g = np.array([0, 0, 9.81])
    cube_weight_force = cube_mass * g

    # Add upward force at both hands to support cube
    wrenchL.linear += cube_weight_force / 2
    wrenchR.linear += cube_weight_force / 2

    tau_grasp = J_R.T @ wrenchR.vector - J_L.T @ wrenchL.vector

    # # Add grasp force to overall tau and step
    tau += tau_grasp 
    # if int(tcurrent*1000) % 10 == 0:
    #     print(f"a[8]: {a[8]}, a[14]: {a[14]}. q[8]: {q[8]}, q[14]: {q[14]}")
    #     print(f"q_err[8]: {q_err[8]}, q_err[14]: {q_err[14]}. tau[8]: {tau[8]}, tau[14]: {tau[14]}")

    sim.step(tau)

    # Recursive loop to drag q back onto the path if it strays too far from 
    # obstacleAvoidanceTau(sim, robot, trajs, tcurrent, tmax, cube, dt)


def runControlLaw(sim, robot, cube, path, num_of_steps, max_time):
    
    trajs, time_steps = getTrajBezier(robot, cube, path, max_time, num_of_steps)

    tcurrent = 0.
    dt = max_time/(len(time_steps))

    while tcurrent < max_time:
        controllaw(sim, robot, trajs, tcurrent, max_time, cube, gains_high, dt)
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




def replanner(sim, robot, cube):
    q0, successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe, successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    if successinit and successend:
        path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    else:
        print("No path available")
        return
    max_time = 1
    num_of_steps = 1000

    runControlLaw(sim, robot, cube, path, num_of_steps, max_time)
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
    #path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    #np.savetxt('path_long.txt', path)
    path = np.loadtxt('path.txt')

    displaypath(robot, cube, path, 0.01, viz)

    #setting initial configuration
    sim.setqsim(q0)
    sim.setTorqueControlMode()

    # COLLISION_THRESHOLD = min(collisionDistance(robot, q) for q in path)

    max_time = 1
    num_of_steps = 600
    
    int_err = np.zeros_like(path[0])

    runControlLaw(sim, robot, cube, path, num_of_steps, max_time)
    