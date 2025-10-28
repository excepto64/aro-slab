import numpy as np

def trajectory(path, T):
    N = len(path)
    out = []
    for n in range(N):
        time = n/(N-1) * T
        out.append(time)
    return out

        


if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path, path_length = computepath(robot, cube, viz, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    print(len(path))
    T = 10
    print(trajectory(path, T))


