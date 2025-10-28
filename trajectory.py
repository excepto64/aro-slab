import pinocchio as pin
import numpy as np
from pinocchio.utils import pinv
from config import EPSILON, DT



def getTrajectory(robot, cube, path):
    t_start = 0
    t_end = 1
    dt = t_end/len(path)
    time_steps = np.arange(t_start, t_end + dt, dt)

    

    for i, t in enumerate(time_steps):
        q_current_goal = path[i]

       
        # pin.computeJointJacobians(robot.model, robot.data, q_goal_current) 

        q_current = robot.data.q.copy()
        pin.framesForwardKinematics(robot.model, robot.data, q_current)

        o_J = pin.computeFrameJacobian(robot.model, robot.data, q_current)
        error = pin.log(q_current_goal.inverse()*q_current).vector

        while error > EPSILON:

            # Control law for multiple tasks
            vq = pinv(o_J) @ error
        
            q = pin.integrate(robot.model, q, vq * DT)
            
              # Implement trajectory following logic here


    return


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath, displaypath
    
    robot, cube, viz = setupwithmeshcat()
    
    
    # q = robot.q0.copy()
    # q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    # qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    # if not(successinit and successend):
    #     print ("error: invalid initial or end configuration")
    
    # path = computepath(robot, cube, viz, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
    path = [3, 3, 2, 3]  # Replace with actual path

    getTrajectory(robot,cube,path)
