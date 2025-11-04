#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, jointlimitsviolated
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
import time
from scipy.optimize import fmin_bfgs
from tools import setcubeplacement


def cost(q, robot, targetL, targetR):
    """
    Computes the squared euclidian distance of the left effector from the left
    hook and the right effector from the right hook, summed together as a cost
    function for the optimisation algorithm.
    Args:
        q (numpy.ndarray): The robot's configuration.
        robot (pin.RobotWrapper): The robot.
        targetL (pin.SE3): The left hook of the cube.
        targetR (pin.SE3): The right hook of the cube.
    Returns:
        float: The total cost of the configuration, compared to the desired 
        grasping configuration.
    """
     # Get fresh data instance and compute forward kinematics and frames.
    data = robot.data.copy()
    pin.forwardKinematics(robot.model, data, q)
    pin.updateFramePlacements(robot.model, data)
    # Get the end effector frames.
    frameL = robot.model.getFrameId(LEFT_HAND)
    frameR = robot.model.getFrameId(RIGHT_HAND)
    effL = data.oMf[frameL]
    effR = data.oMf[frameR] 
    # Compute the squared Euclidian distance between the effector and target.
    errL = norm(pin.log(effL.inverse() * targetL).vector)**2
    errR = norm(pin.log(effR.inverse() * targetR).vector)**2
    return errR + errL

def computeqgrasppose(robot, qcurrent, cube, cubetarget):
    '''
    Return a collision free configuration grasping a cube at a specific
    location and a success flag.
    Args:
        robot (pin.RobotWrapper): The robot.
        qcurrent (numpy.ndarry): The robots current configuration.
        cube (pin.RobotWrapper): The cube.
        cubetarget (pin.SE3): The position of the cube. 
    Returns:
        (q, success) (numpy.ndarray, bool): The configuration of the robot
        grasping the cube and whether grasping was successful.
    '''
    setcubeplacement(robot, cube, cubetarget)
    targetL = getcubeplacement(cube, LEFT_HOOK) #placement of the left hand hook
    targetR = getcubeplacement(cube, RIGHT_HOOK) #placement of the right hand hook
    # Get grasping position and check it is valid.
    q, _, _, _, _, _, warn = fmin_bfgs(cost, qcurrent, args=(robot, targetL, targetR), full_output=True, disp=False)
    success = warn == 0 and not collision(robot, q) and not jointlimitsviolated(robot, q)
    return q, success
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    print(CUBE_PLACEMENT.__class__)
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    updatevisuals(viz, robot, cube, q0)
    
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    updatevisuals(viz, robot, cube, q0)