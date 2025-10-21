#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv, norm

from config import LEFT_HAND, RIGHT_HAND
import time
from math import ceil
from tools import getcubeplacement, setcubeplacement
from setup_meshcat import updatevisuals
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


discretisationsteps_newconf = 200 #To tweak later on
discretisationsteps_validedge = 200 #To tweak later on

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal, robot, cube, viz):
    k = 1000
    delta_q = 1. #To tweak later on
    G, success = rrt(qinit, qgoal, k, delta_q, robot, cube, cubeplacementq0, cubeplacementqgoal, viz)
    return getpath(G)


def displaypath(robot,path,dt,viz):
    for q0, q1 in zip(path[:-1],path[1:]):
        displayedge(q0,q1, viz, robot, cube)

def displayedge(q0,q1, viz, robot, cube, vel=2.): #vel in sec.    
    '''Display the path obtained by linear interpolation of q0 to q1 at constant velocity vel'''
    dist = distance(q0[0],q1[0])
    duration = dist / vel    
    nframes = ceil(48. * duration)
    f = 1./48.
    for i in range(nframes-1):
        setcubeplacement(robot, cube, se3_lerp(q0[1], q1[1], float(i)/nframes))
        viz.display(lerp(q0[0],q1[0],float(i)/nframes))
        updatevisuals(viz, robot, cube)
        time.sleep(f)
    viz.display(q1)
    time.sleep(f)

def sample_config(robot, cube, cube_start, cube_target, viz):
    # Sample configurations until a successful one is found
    dist = norm(cube_target.translation - cube_start.translation)
    upper = np.maximum(cube_start.translation, cube_target.translation) + 0.5*dist
    lower = np.minimum(cube_start.translation, cube_target.translation) - 0.5*dist
    success = False
    while not success:
        rand = np.random.random(3)
        cube_pos = np.array([0.0,0.0,0.0,0.0,0.0,0.0,1.0])
        cube_pos[0:3] += (upper - lower) * rand + lower
        pos = pin.XYZQUATToSE3(cube_pos)
        setcubeplacement(robot, cube, pos)
        q, success = computeqgrasppose(robot, robot.q0, cube, pos, viz)

    return q, pos

def coll(q):
     '''Return true if q in collision, false otherwise.'''
     pin.updateGeometryPlacements(robot.model,robot.data,robot.collision_model,robot.collision_data,q)
     return pin.computeCollisions(robot.collision_model,robot.collision_data,False)

def distance(q1,q2):    
     '''Return the euclidian distance between two configurations'''
     return norm(q2 - q1)
        
def NEAREST_VERTEX(G,q_rand):
     '''returns the index of the Node of G with the configuration closest to q_rand  '''
     res = np.inf
     ind = -1
     for i in range(len(G)):
          if distance(q_rand, G[i][1]) < res:
               ind = i
               res = distance(q_rand, G[i][1])
     return ind

def ADD_EDGE_AND_VERTEX(G,parent,q,pos):
    G += [(parent,q,pos)]

def lerp(q0,q1,t):    
    """Linear interpolation"""
    return q0 * (1 - t) + q1 * t

def se3_lerp(M1, M2, t):
    """
    Geodesic interpolation using Pinocchio's exp/log maps.
    """
    # Compute relative transformation
    M_rel = M1.inverse() * M2
    
    # Get the tangent vector (twist)
    nu = pin.log(M_rel)  # Returns Motion (6D twist)
    
    # Scale by interpolation parameter
    nu_interp = t * nu
    
    # Exponentiate back to SE3
    M_interp_rel = pin.exp(nu_interp)
    
    # Apply to initial pose
    return M1 * M_interp_rel

def NEW_CONF(pos_near,pos_rand,discretisationsteps, robot, cube, delta_q = None):
    '''Return the closest configuration q_new such that the path q_near => q_new is the longest
    along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q'''
    dist = distance(pos_rand.translation, pos_near.translation)
    pos_end = pos_rand.copy()
    if delta_q is not None and dist > delta_q:
        pos_end = se3_lerp(pos_near, pos_rand, delta_q/dist)
    dt = 1 / discretisationsteps
    for i in range(discretisationsteps):
        pos_new = se3_lerp(pos_near, pos_end, dt*i)
        q_new, _ = computeqgrasppose(robot, robot.q0, cube, pos_new)
        if coll(q_new):
            return se3_lerp(pos_near, pos_end, dt*(i-1))
    return pos_end

def NEW_CONF_q(q_near,q_rand,discretisationsteps, delta_q = None):
    '''Return the closest configuration q_new such that the path q_near => q_new is the longest
    along the linear interpolation (q_near,q_rand) that is collision free and of length <  delta_q'''
    dist = distance(q_rand, q_near)
    q_end = q_rand.copy()
    if delta_q is not None and dist > delta_q:
        q_end = lerp(q_near, q_rand, delta_q/dist)
    dt = 1 / discretisationsteps
    for i in range(discretisationsteps):
        q_new = lerp(q_near, q_end, dt*i)
        if coll(q_new):
            return lerp(q_near, q_end, dt*(i-1))
    return q_end

def VALID_EDGE(q_new,q_goal,discretisationsteps):
    return norm(q_goal - NEW_CONF_q(q_new, q_goal,discretisationsteps)) < 1e-3

def rrt(q_init, q_goal, k, delta_q, robot, cube, cubeplacementq0, cubeplacementqgoal, viz):
    G = [(None,q_init,cubeplacementq0)]
    for _ in range(k):
        q_rand, pos_rand = sample_config(robot, cube, cubeplacementq0, cubeplacementqgoal, viz)
        #print("Random: ", q_rand)   
        q_near_index = NEAREST_VERTEX(G,q_rand)
        pos_near = G[q_near_index][2]        
        pos_new = NEW_CONF(pos_near,pos_rand,discretisationsteps_newconf, robot, cube, delta_q = None)    
        q_new, _ = computeqgrasppose(robot, robot.q0, cube, pos_new)
        ADD_EDGE_AND_VERTEX(G,q_near_index,q_new, pos_new)
        #print("Added edge: ", q_near, q_new)
        if VALID_EDGE(q_new,q_goal,discretisationsteps_validedge):
            print ("Path found!")
            ADD_EDGE_AND_VERTEX(G,len(G)-1,q_goal, cubeplacementqgoal)
            return G, True
        if _ % 100 == 0:
            print("RRT round ", _)
    print("path not found")
    return G, False

def getpath(G):
    path = []
    node = G[-1]
    while node[0] is not None:
        path = [node[1:3]] + path
        node = G[node[0]]
    path = [G[0][1:3]] + path
    return path

if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    print(CUBE_PLACEMENT.__class__)
    q = robot.q0.copy()
    updatevisuals(viz, robot, cube, q)
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    updatevisuals(viz, robot, cube, q0)
    time.sleep(0.5)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    updatevisuals(viz, robot, cube, qe)
    time.sleep(0.5)

    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, robot, cube, viz)
    
    displaypath(robot,path,cube,dt=0.5, viz=viz) #you ll probably want to lower dt
    time.sleep(2)
    displaypath(robot,path,cube,dt=0.5, viz=viz) #you ll probably want to lower dt