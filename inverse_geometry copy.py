#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from scipy.optimize import fmin_bfgs
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, jointlimitsviolated
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON, DT
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, OBSTACLE_PLACEMENT
from setup_meshcat import updatevisuals

from tools import setcubeplacement

# def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
#     '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
#     setcubeplacement(robot, cube, cubetarget)
    
#     q = qcurrent.copy()
#     takesTooLong = 0
#     takesTooLongLimit = 15000
    
#     # Assign goals
#     oMleft_hook = getcubeplacement(cube, LEFT_HOOK)
#     oMright_hook = getcubeplacement(cube, RIGHT_HOOK)

    
#     # Assign indexes 
#     IDX_L = robot.model.getFrameId(LEFT_HAND)
#     IDX_R = robot.model.getFrameId(RIGHT_HAND)
    
#     atGraspPose = False
    
#     while(not atGraspPose):
#         # Get values in robot.data
#         pin.framesForwardKinematics(robot.model, robot.data, q)
#         pin.computeJointJacobians(robot.model,robot.data,q) 
        
#         # Left
#         oMleft = robot.data.oMf[IDX_L]
#         o_Jleft = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_L)
#         error_left = pin.log(oMleft.inverse()*oMleft_hook).vector
        
#         #Right
#         oMright = robot.data.oMf[IDX_R]
#         o_Jright = pin.computeFrameJacobian(robot.model, robot.data, q, IDX_R)
#         error_right = pin.log(oMright.inverse()*oMright_hook).vector
        
        
#         # Control law for multiple tasks
#         vq = pinv(o_Jleft) @ error_left
#         P = np.eye(robot.nv)-pinv(o_Jleft) @ o_Jleft
#         vq -= pinv(o_Jright @ P) @ (-error_right - o_Jright @ vq)
        
#         q = pin.integrate(robot.model, q, vq * DT)
        
# #         updatevisuals(viz, robot, cube, q)
        
#         # Check the current pose is within the threshold
#         error_check = np.max(error_left) + np.max(error_right)
#         if (error_check < EPSILON):
            
#             # Ensure q is not colliding with anything
#             if collision(robot, q):
#                 return q, atGraspPose
                
#             # Ensure joint limits are not violated
#             elif jointlimitsviolated(robot, q):
#                 return q, atGraspPose
                
#             else:
#                 atGraspPose = True
#                 return q, atGraspPose
            
#         if takesTooLong > takesTooLongLimit:
# #             print("Took too long")
#             return q, atGraspPose
            
#         takesTooLong+= 1
        
        
def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    setcubeplacement(robot, cube, cubetarget)
    targetL = getcubeplacement(cube, LEFT_HOOK) #placement of the left hand hook
    targetR = getcubeplacement(cube, RIGHT_HOOK) #placement of the right hand hook
#     print(targetL)
#     print(targetR)
#     print(cubetarget)
    q, _, _, _, _, _, warn = fmin_bfgs(cost, qcurrent, args=(robot, targetL, targetR), full_output=True, disp=False)
    return q, warn == 0 and not collision(robot, q) and not jointlimitsviolated(robot, q)



def cost(q, robot, targetL, targetR):
    data = robot.data.copy()
    pin.forwardKinematics(robot.model, data, q)
    pin.updateFramePlacements(robot.model, data)
    frameL = robot.model.getFrameId(LEFT_HAND)
    frameR = robot.model.getFrameId(RIGHT_HAND)
    effL = data.oMf[frameL]
    effR = data.oMf[frameR]
    errL = np.linalg.norm(pin.log(effL.inverse()*targetL).vector)**2
    errR = np.linalg.norm(pin.log(effR.inverse()*targetR).vector)**2
    
    return errL + errR
    
# Code taken from Tutorial 3 (invkine)
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q0)
    
    
    
