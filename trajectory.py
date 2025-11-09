import pinocchio as pin
import numpy as np

from config import EPSILON, DT, LEFT_HAND
from tools import setcubeplacement, getcubeplacement
from bezier import Bezier
import time

from setup_meshcat import updatevisuals


def getTrajBezier(robot, cube, path, max_time, num_of_steps):
    print("Getting Bezier Trajectory")
    t_start = 0
    t_end = max_time

    step_size = max_time / num_of_steps
    time_steps = np.arange(t_start, t_end+step_size, step_size)


    bezierPoints = getBezierPoints(path, num_of_steps)

    trajs = []

    # Make one single Bezier curve with multiple points in it from getShortenedPath
    # Then iterate through time steps in the control law!!!!!
    trajs = Bezier(bezierPoints, 0, max_time)
        
    return trajs, time_steps


def getBezierPoints(path, num_of_steps):
    bezierPath = np.empty([num_of_steps+3, len(path[0])])
    bezierPath[0] = path[0]
    bezierPath[1:-1] = getShortenedPath(path, num_of_steps)
    bezierPath[-1] = path[-1]
    
    return bezierPath


def getShortenedPath(path, num_of_steps):
    '''
    Get a shortened version of the path, with each segment being equidistant
    '''
    shortened_path = np.empty([num_of_steps+1, len(path[0])])
    for i in range(num_of_steps):
        path_idx = (int) ((len(path) / num_of_steps) * i)
        shortened_path[i] = path[path_idx]

    shortened_path[-1] = path[-1]

    return shortened_path


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath
    from bezier import Bezier
    
    robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6003")
    
    q = robot.q0.copy()
    pin.framesForwardKinematics(robot.model,robot.data,q)

    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    # path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    path = np.loadtxt('path.txt')


    P, I, D = 10, 1, 1
    max_time = 30
    num_of_steps = 30
    step_size = max_time / num_of_steps

    # trajs, time_steps = getTrajectory1(robot, cube, path, max_time, P, I, D)

    trajs, time_steps = getTrajBezier(robot, cube, path, max_time, num_of_steps)

    # Testing that the Belzier curve gives a path 
    trajpath = []
    traj_time_steps = np.arange(0, max_time, 0.1)
    for t in traj_time_steps:
        trajpath += [trajs(t)]

    displaypath(robot, cube, trajpath, 1/10, viz)
