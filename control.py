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
KiHP = 0.2 * KpHP
KdHP = 2 * np.sqrt(KpHP)
grasp_forceHP = 1800

COLLISION_THRESHOLD = 1e-3


# Constants:
Kp = 25
Ki = 0.01 * Kp 
Kd = 2.5 * np.sqrt(Kp)
grasp_force = 100*0
INT_DECAY = 0.98
PID_DECAY = 0.95

def controllaw(sim, robot, trajs, tcurrent, tmax, cube, dt = DT):
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

    
    if tcurrent > t_lowering:
        Kp_phase = Kp * PID_DECAY
        Ki_phase = Ki * PID_DECAY
        Kd_phase = Kd * PID_DECAY
        force_phase = grasp_force
    else:
        Kp_phase = Kp 
        Ki_phase = Ki 
        Kd_phase = Kd 
        force_phase = grasp_force 

    # a = a_desired + (Kp_phase * q_err) + (Kd_phase*v_err) + (Ki_phase * int_err)

    # tau_g = pin.computeGeneralizedGravity(robotsim.model, robotsim.data, q)

    # # Calculate basic Tau
    # tau = pin.rnea(robot.model, robot.data, q, vq, a)#, [robomass]) #, [f_ext]) <---- Find adequate exterior forces e.g. mass of the robot
    # tau_m = robot.data.tau.copy()

    # tau_model = pin.rnea(robot.model, robot.data, q, vq, a_desired)

    # PID correction torque
    # tau_control = (Kp_phase * q_err) + (Kd_phase * v_err) + (Ki_phase * int_err)

    # tau = tau_model + tau_control

    tau = pin.rnea(robot.model, robot.data, q, np.zeros_like(q), np.zeros_like(q))

    # Add "grasp force" so the cube does not get dropped
    # For left hand
    idx_L = robot.model.getFrameId(LEFT_HAND)
    oML = robot.data.oMf[idx_L]
    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    left_err = oMcubeL.translation - oML.translation
    # trans_err = oMcubeL.translation - oML.translation
    # rot_err = 0.5 * pin.log3(oMcubeL.rotation @ oML.rotation.T)
    # wrenchL = pin.Motion(np.hstack((trans_err * force_phase, rot_err)) )
    wrenchL = pin.Motion(left_err * force_phase, np.zeros(3))
    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, idx_L, pin.LOCAL_WORLD_ALIGNED)

    # For right hand
    idx_R = robot.model.getFrameId(RIGHT_HAND)
    oMR = robot.data.oMf[idx_R]
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    right_err = oMcubeR.translation - oMR.translation
    # trans_err = oMcubeR.translation - oMR.translation
    # rot_err = 0.5 * pin.log3(oMcubeR.rotation @ oMR.rotation.T)
    # wrenchR = pin.Motion(trans_err * force_phase, rot_err)
    wrenchR = pin.Motion(right_err * force_phase, np.zeros(3))
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, idx_R, pin.LOCAL_WORLD_ALIGNED)

    Rot_err = 0.5 * pin.log3(oMcubeL.rotation @ oML.rotation.T)
    R_err = 0.5 * pin.log3(oMcubeR.rotation @ oMR.rotation.T)


    tau_grasp = J_R.T @ wrenchR.vector - J_L.T @ wrenchL.vector

    # Add grasp force to overall tau and step
    print("q_err_norm:", np.linalg.norm(q_err), "v_err_norm:", np.linalg.norm(v_err))
    print("int_err (max/min):", np.max(int_err), np.min(int_err))
    print("tau_grasp (first 6):", tau_grasp[:6])
    print("tau_total (first 6):", (tau + tau_grasp)[:6])

    # tau += tau_grasp 
    sim.step(tau)



def controllawHighPID(sim, robot, trajs, tcurrent, tmax, cube, dt = DT):
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

    
    if tcurrent > t_lowering:
        Kp_phase = KpHP * PID_DECAY
        Ki_phase = KiHP * PID_DECAY
        Kd_phase = KdHP * PID_DECAY
        force_phase = grasp_forceHP
    else:
        Kp_phase = KpHP
        Ki_phase = KiHP 
        Kd_phase = KdHP
        force_phase = grasp_forceHP

    a = a_desired + (Kp_phase * q_err) + (Kd_phase*v_err) + (Ki_phase * int_err)

    # Calculate basic tau
    # tau = pin.rnea(robot.model, robot.data, q, vq, a)#, [robomass]) #, [f_ext]) <---- Find adequate exterior forces e.g. mass of the robot
    tau = pin.rnea(robot.model, robot.data, q, vq, a)

    '''
    Extension for obstacle avoidance:
    Access obstacles from config (or pin.getObstacles type function to get all 
    obstacles in a sim / environment) and check q against it every step. If 
    for any step q is within some threshold of any obstacle, access that 
    obstacles q, use similar process to grasp force to find a small tau to add 
    to the final tau to move away from the obstacle.
    '''    


    # Add "grasp force" so the cube does not get dropped
    # For left hand
    idx_L = robot.model.getFrameId(LEFT_HAND)
    oML = robot.data.oMf[idx_L]
    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    trans_err = oMcubeL.translation - oML.translation
    rot_err = 0.5 * pin.log3(oMcubeL.rotation @ oML.rotation.T)
    wrenchL = pin.Motion(np.hstack((trans_err * force_phase, rot_err)) )
    J_L = pin.computeFrameJacobian(robot.model, robot.data, q, idx_L, pin.LOCAL_WORLD_ALIGNED)

    # For right hand
    idx_R = robot.model.getFrameId(RIGHT_HAND)
    oMR = robot.data.oMf[idx_R]
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    trans_err = oMcubeR.translation - oMR.translation
    rot_err = 0.5 * pin.log3(oMcubeR.rotation @ oMR.rotation.T)
    wrenchR = pin.Motion(trans_err * force_phase, rot_err)
    J_R = pin.computeFrameJacobian(robot.model, robot.data, q, idx_R, pin.LOCAL_WORLD_ALIGNED)

    tau_grasp = J_R.T @ wrenchR.vector - J_L.T @ wrenchL.vector

    # Add grasp force to overall tau and step
    tau += tau_grasp 
    sim.step(tau)

    # Recursive loop to drag q back onto the path if it strays too far from 
    # obstacleAvoidanceTau(sim, robot, trajs, tcurrent, tmax, cube, dt)


def runControlLaw(sim, robot, cube, path, num_of_steps, max_time):
    
    trajs, time_steps = getTrajBezier(robotsim, cubesim, path, max_time, num_of_steps)

    tcurrent = 0.
    dt = max_time/(len(time_steps))

    while tcurrent < max_time:
        controllawHighPID(sim, robot, trajs, tcurrent, max_time, cube, dt)
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
        controllawHighPID(sim, robot, trajs, tcurrent, tmax, cube, dt)
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
    q0, successinit = computeqgrasppose(robotsim, robotsim.q0, cubesim, CUBE_PLACEMENT, None)
    qe, successend = computeqgrasppose(robotsim, robotsim.q0, cubesim, CUBE_PLACEMENT_TARGET,  None)
    if successinit and successend:
        path = computepath(robotsim, cubesim, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
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
    
    robotsim, sim, cubesim = setupwithpybullet()
    robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6003")
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath
    from trajectory import getTrajBezier
    
    q0,successinit = computeqgrasppose(robotsim, robotsim.q0, cubesim, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robotsim, robotsim.q0, cubesim, CUBE_PLACEMENT_TARGET,  None)
    # path = computepath(robotsim, cubesim, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    path = np.loadtxt('path.txt')

    # path = computepath(robot, cube, q0, qrightup, CUBE_PLACEMENT, CUBE_PLACEMENT_RIGHT_UP)

    #setting initial configuration
    sim.setqsim(q0)
    sim.setTorqueControlMode()

    # COLLISION_THRESHOLD = min(collisionDistance(robot, q) for q in path)

    max_time = 1
    num_of_steps = 1000
    
    int_err = np.zeros_like(path[0])

    runControlLaw(sim, robot, cube, path, num_of_steps, max_time)
    