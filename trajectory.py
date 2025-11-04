import pinocchio as pin
import numpy as np

from config import EPSILON, DT, LEFT_HAND
from tools import setcubeplacement, getcubeplacement
import time




def getTrajectory(robot, cube, viz, path):
    print("getting Trajectory")
    t_start = 0
    t_end = 1
    dt = t_end/len(path)
    time_steps = np.arange(t_start, t_end + dt, dt)
    P, I, D = 1, 1, 1


    trajectories = []


    for i, t in enumerate(time_steps):
        print("At time step" , i, "of", len(time_steps))
        q_current = path[i]
        q_current_goal = path[i+1]

        trajectories.append(stepThroughPath(robot, cube, viz, q_current, q_current_goal, P, I, D))

    return trajectories


def stepThroughPath(robot, cube, viz, q_current, q_current_goal, k_p, k_i, k_d):

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
        q_current = pin.integrate(robot.model, q_current, u*DT)
        trajectories.append(u)

        data = robot.data.copy()  # Fresh data instance
        pin.forwardKinematics(robot.model, data, q_current)
        pin.updateFramePlacements(robot.model, data)  # Important!

        # For visuialiser
        effL = data.oMf[robot.model.getFrameId(LEFT_HAND)] 
        cube_pos = getcubeplacement(cube)
        eMc = effL.inverse() * cube_pos
        
        viz.display(q_current)
        setcubeplacement(robot, cube, effL * eMc)

        q_prev = q_current

    return trajectories






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
    
    path = computepath(robot, cube, viz, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    

    trajs = getTrajectory(robot, cube, viz, path)
    print(trajs)