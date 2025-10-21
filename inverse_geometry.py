#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
import time
from scipy.optimize import fmin_bfgs
from tools import setcubeplacement


def cost(q, robot, targetL, targetR):
    data = robot.data.copy()  # Fresh data instance
    pin.forwardKinematics(robot.model, data, q)
    pin.updateFramePlacements(robot.model, data)  # Important!
    frameL = robot.model.getFrameId(LEFT_HAND)
    effL = data.oMf[frameL]
    errL = norm(pin.log(effL.inverse() * targetL).vector)**2
    frameR = robot.model.getFrameId(RIGHT_HAND)
    effR = data.oMf[frameR] 
    errR = norm(pin.log(effR.inverse() * targetR).vector)**2
    return errR + errL

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)
    targetL = getcubeplacement(cube, LEFT_HOOK) #placement of the left hand hook
    targetR = getcubeplacement(cube, RIGHT_HOOK) #placement of the right hand hook
    q, _, _, _, _, _, warn = fmin_bfgs(cost, qcurrent, args=(robot, targetL, targetR), full_output=True, disp=False)
    coll = collision(robot, q)
    success = warn == 0 and not coll
    return q, success
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    updatevisuals(viz, robot, cube, q0)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    updatevisuals(viz, robot, cube, q0)
    
    
    
