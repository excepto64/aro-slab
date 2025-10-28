#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from tools import setupwithmeshcat, setcubeplacement, getcubeplacement
from inverse_geometry import computeqgrasppose
from config import LEFT_HAND
import time

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(robot, cube, viz, qinit, qgoal, cubeplacementq0, cubeplacementqgoal):
    
    numberOfSamples4Search = 1000
    deltaq = 3
    interpolation_steps = 200
    
    # Should RRT pass robot q 
    G, pathFound = RRT(robot, cube, viz, qinit, qgoal, cubeplacementq0, cubeplacementqgoal, numberOfSamples4Search, deltaq)
    
    if pathFound:
        print("Path found with", len(G), "nodes")
        return getPathFromGraph(G, robot, cube, viz, interpolation_steps)
    else: 
        print("No path found")
        return []
    

    
def RRT(robot, cube, viz, qinit, qgoal, cubeq0, cubeqgoal, k, deltaq):
    G = [(None, qinit, cubeq0, cubeq0.translation, cubeq0.rotation)]
    
    steps4NewConf = 200 #To tweak later on
    steps4ValidEdge = 200 #To tweak later on
    delta_q = None
    
    print("Computing path...")

    for i in range(k):
        
        qrand, cuberand = randomSampleRobotConfig(robot, cube, viz, cubeq0, cubeqgoal, G)
        qnear_index = getNearestVertex(G, qrand)
        cubenear = G[qnear_index][2]      
        qnew, cubenew = newConfigNearObstacle(robot, qrand, cubenear, cuberand, steps4NewConf, delta_q, cube, viz)    
        addValidEdgeAndVertex(G, qnear_index, qnew, cubenew)
        
        if validEdgeSE3(robot, qnew, cube, viz, cubenew, cubeqgoal, steps4ValidEdge):
            addValidEdgeAndVertex(G,len(G)-1,qgoal, cubeqgoal)
            return G, True
        
#         if i%10==0:
#             print("|", end="")
#         else:
#             print("-", end="")
#         if i%100==0 and i!=0:
#             print("\n|", end="")
            
    return G, False
    

    
def randomSampleRobotConfig(robot, cube, viz, cubeq0, cubeqgoal, G):
    
    q0 = cubeq0.translation
    qgoal = cubeqgoal.translation
    cubeq0rot = pin.Quaternion(cubeq0.rotation)
    cubeqgoalrot = pin.Quaternion(cubeqgoal.rotation)
    validConfig = False
    q = robot.q0.copy()
    
    # Get sample space 
    centrepoint = np.array(0.5 * (q0 + qgoal))
    radius = (np.linalg.norm(q0 - centrepoint)) 
    
    while not validConfig:
        # Random rotational sample between q0 and q1 rotation
        t = np.random.rand()
        randomRotation = pin.Quaternion.slerp(cubeq0rot, t, cubeqgoalrot)
        
        # Random sample in 3D space
        randomSample = centrepoint+(np.random.rand(3)*2-1) * radius
        
        RANDOM_CUBE_PLACEMENT = pin.SE3(randomRotation, randomSample)
  
        if not duplicateCube(G, RANDOM_CUBE_PLACEMENT):
            config, validConfig = computeqgrasppose(robot, q, cube, RANDOM_CUBE_PLACEMENT, viz)
                  
    return config, RANDOM_CUBE_PLACEMENT


def duplicateCube(G, cubeQ, tol=1e-2):
    cubeTrans = cubeQ.translation
    cubeRot = cubeQ.rotation
    for (_, _, _, trans, rot) in G:
        euclid_dist = distance(trans, cubeTrans)
        rotation_dist = distance(rot, cubeRot)
        if euclid_dist < tol and rotation_dist < tol:
            return True
    return False



def addValidEdgeAndVertex(G, parent, q, cube):
    G += [(parent, q, cube, cube.translation, cube.rotation)]


def getNearestVertex(G, qrand):
    min_dist = np.inf
    idx=-1
    for (i, node) in enumerate(G):
        dist = distance(node[1] ,qrand)
        if dist < min_dist:
            min_dist = dist
            idx = i
    return idx


def newConfigNearObstacle(robot, q, cubenear, cuberand, steps4NewConf, delta_q, cube, viz):
    dist = distanceSE3(cubenear, cuberand)
    qprevious, _ = computeqgrasppose(robot, q, cube, cubenear, viz)
    cubeprevious = cubenear
    
    if delta_q is not None and dist > delta_q:
        cubeprevious = interpolationSE3(cubenear, cuberand, delta_q/dist)
        qprevious, validConfig = computeqgrasppose(robot, q, cube, cubeprevious, viz)
        
    conf_path, cube_path = interPath(robot, q, cube, viz, cubenear, cuberand, steps4NewConf)
    
    if len(conf_path) != 0 and len(cube_path) != 0: 
        return conf_path[-1], cube_path[-1]
    else:
        return qprevious, cubeprevious
    
    
def interpolationSE3(q0, q1, t):
    p0 = q0.translation
    p1 = q1.translation
    p = p0 * (1 - t) + p1 * t
    
    q0rot = pin.Quaternion(q0.rotation)
    q1rot = pin.Quaternion(q1.rotation)
    
    qrot = pin.Quaternion.slerp(q0rot, t, q1rot)
    qrot.normalize()
    return pin.SE3(qrot, p)


def interPath(robot, q_start, cube, viz, q0, q1, steps):
    inter_path_configs = []
    inter_path_cubes = []
    q_current = q_start.copy()

    for step in range(steps):
        cubeq = interpolationSE3(q0, q1, step / steps)
        q_next, validConfig = computeqgrasppose(robot, q_current, cube, cubeq, viz)

        if not validConfig:
            break  
        inter_path_configs.append(q_next)
        inter_path_cubes.append(cubeq)
        q_current = q_next 

    return inter_path_configs, inter_path_cubes
    

def validEdgeSE3(robot, q, cube, viz, cubeq0, cubeqgoal, steps):
    for step in range(steps):
        cubeq = interpolationSE3(cubeq0, cubeqgoal, step/steps)
        q, validConfig = computeqgrasppose(robot, q, cube, cubeq, viz)
        if not validConfig:
            return False
    return True


def getPathFromGraph(G, robot, cube, viz, steps):
    path = []
    config_path = []
    cube_path = []
    node = G[-1]
    
    while node[0] is not None:
        config_path = [node[1]] + config_path
        cube_path = [node[2]] + cube_path
        node = G[node[0]]
        
    config_path = [node[1]] + config_path
    cube_path = [node[2]] + cube_path
        
    for _ in range(int(len(G)/2)):
        config_path, cube_path = shortcut(config_path, cube_path, robot, node[1], cube, viz, steps)
        
    for i in range(len(config_path) - 1):
        path += [config_path[i]]
        configs, _ = interPath(robot, node[1], cube, viz, cube_path[i], cube_path[i+1], steps)

        path += configs
        
        
    path = path + [config_path[-1]]
    return path


# Shortcut with cubes? 
def shortcut(path, cube_path, robot, q, cube, viz, steps):
    for i, cubeq in enumerate(cube_path):
        for j in reversed(range(i+1,len(cube_path))):
            cubeq2 = cube_path[j]
            if validEdgeSE3(robot, q, cube, viz, cubeq, cubeq2, steps):
                path = path[:i+1]+ path[j:]
                cube_path = cube_path[:i+1]+ cube_path[j:]
                return path, cube_path
    return path, cube_path


def distance(q1, q2):
    distance = np.linalg.norm(q2-q1)
    return distance


def distanceSE3(q1, q2):
    distance = pin.log(q1.inverse()*q2).vector
    return distance


def displaypath(robot,cube,path,dt,viz):
    q_new = path[-1].copy()
    data = robot.data.copy()  # Fresh data instance
    pin.forwardKinematics(robot.model, data, q_new)
    pin.updateFramePlacements(robot.model, data)  # Important!
    frameL = robot.model.getFrameId(LEFT_HAND)
    effL = data.oMf[frameL] 
    cube_pos = getcubeplacement(cube)
    eMc = effL.inverse() * cube_pos

    for q in path:
        viz.display(q)
        setcubeplacement(robot, cube, getcube(robot, q, eMc))
        time.sleep(dt)
        
        
def getcube(robot, q, eMc):
    data = robot.data.copy()  # Fresh data instance
    pin.forwardKinematics(robot.model, data, q)
    pin.updateFramePlacements(robot.model, data)  # Important!
    frameL = robot.model.getFrameId(LEFT_HAND)
    effL = data.oMf[frameL] 
    return effL * eMc


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(robot, cube, viz, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    displaypath(robot,cube,path,dt=0.005,viz=viz) #you ll probably want to lower dt