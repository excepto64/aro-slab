import pinocchio as pin
import numpy as np

from config import EPSILON, DT, LEFT_HAND
from tools import setcubeplacement, getcubeplacement
from bezier import Bezier
import time

from setup_meshcat import updatevisuals



def getTrajectory1(robot, cube, path, max_time, P, I, D):
    print("Getting Trajectory")
    t_start = 0
    t_end = max_time

    # len(path) - 2 to make len(path) 1 longer than len(time_steps) 
    dt = t_end/(len(path)-2)
    time_steps = np.arange(t_start, t_end + dt, dt)

    trajectories = np.empty([len(time_steps), 2, len(path[0])])

    print(trajectories.shape)


    for i in range(len(time_steps)-1):
        # print("At time step" , i, "of", len(time_steps))
        q_current = path[i]
        q_current_goal = path[i+1]

        trajectories[i][0] = q_current
        trajectories[i][1] = q_current_goal - q_current

        # trajectories += [stepThroughPath(robot, cube, q_current, q_current_goal, P, I, D)]

    trajectories[-1][0] = path[-1]
    trajectories[-1][0] = np.zeros(len(path[-1]))
    return trajectories, time_steps


def stepThroughPath(robot, cube, q_current, q_current_goal, k_p, k_i, k_d):

    error = q_current_goal - q_current
    q_prev = q_current
    error_i = np.zeros(error.shape)
    trajectories = []

    while np.sum(np.abs(error))/2 > EPSILON:

        error = q_current_goal - q_current
        error_d = (error - (q_current_goal - q_prev)) / DT       
        error_i += error * DT
                
        u = (k_p * error) + (k_d * error_d) + (k_i * error_i)

        # Do I need to save q_current like this? I should probably sense q somehow
        q_current = pin.integrate(robot.model, q_current, u*DT*5)
        trajectories += [u]

        data = robot.data.copy()  # Fresh data instance
        pin.forwardKinematics(robot.model, data, q_current)
        pin.updateFramePlacements(robot.model, data)  # Important!

        # # For visuialiser
        # effL = data.oMf[robot.model.getFrameId(LEFT_HAND)] 
        # cube_pos = getcubeplacement(cube)
        # eMc = effL.inverse() * cube_pos
        
        # cube_pos =  effL * eMc
        # setcubeplacement(robot, cube, cube_pos)
        # viz.display(q_current)

        q_prev = q_current

    return trajectories




def getTrajBezier(robot, cube, path, max_time, num_of_steps):
    print("Getting Bezier Trajectory")
    t_start = 0
    t_end = max_time

    step_size = max_time / num_of_steps
    time_steps = np.arange(t_start, t_end+step_size, step_size)


    short_path = getShortenedPath(path, num_of_steps)

    trajs = []

    for i in range(len(time_steps)-1):
        trajs += [makeBezTraj(short_path[i], short_path[i+1], 
                              time_steps[i], time_steps[i] + step_size)]

    
    return trajs, time_steps

def getShortenedPath(path, num_of_steps):
    '''
    Get a shortened version of the path, with each segment being equidistant
    '''
    shortened_path = np.empty([num_of_steps+1, len(path[0])])
    for i in range(num_of_steps):
        if i == 0:
            path_idx = 0
        else:
            path_idx = (int) ((len(path) / num_of_steps) * i)
        shortened_path[i] = path[path_idx]

    shortened_path[-1] = path[-1]

    return shortened_path


def makeBezTraj(q0,q1,t0, t1):
        q_of_t = Bezier([q0,q0,q1,q1], t_min=t0, t_max=t1)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t

    


# def displaypath(robot,cube,path,dt, viz):
#     q_new = path[-1].copy()
#     data = robot.data.copy()  # Fresh data instance
#     pin.forwardKinematics(robot.model, data, q_new)
#     pin.updateFramePlacements(robot.model, data)  # Important!
#     frameL = robot.model.getFrameId(LEFT_HAND)
#     effL = data.oMf[frameL] 
#     cube_pos = getcubeplacement(cube)
#     eMc = effL.inverse() * cube_pos



if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath
    
    robot, cube, viz = setupwithmeshcat(url="tcp://127.0.0.1:6003")
    
    q = robot.q0.copy()
    pin.framesForwardKinematics(robot.model,robot.data,q)

    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)


    P, I, D = 10, 1, 1
    max_time = 10
    num_of_steps = 3

    # trajs, time_steps = getTrajectory1(robot, cube, path, max_time, P, I, D)

    trajs, time_steps = getTrajBezier(robot, cube, path, 10, num_of_steps)

    
    # print(len(path))
    # print(len(time_steps))
    # print(len(trajs))

    # print(trajs[1])
    # print(path[0])
    # print(path[0]==path[1])