# Advanced Robotics CW2 Submission

## Software our code has used

Basic libraries from tutorials were used:
- Time
- Numpy
- Scipy
- Pinocchio

No other software is required to run our code.

## Algorithms for each section

### Inverse Geometry

Inverse Geometry uses the bfgs algorithm solver to find a valid grasp position. 

### Path

Path uses RRT to build a graph of nodes where each node corresponds to a configuration q. After finding a path using the RRT method, a shortcut algorithm based on the tutorial 4 shortcut has been imporved upon for our implementation. Path also uses a duplicate cube position checker to ensure cube positions are sufficiently spread to increase pathfinder efficiency. 

### Dynamics

Control uses the Bezier approach for calculating trajectories. Our implementation segments the path based on a set of time intervals, and creates a *single* bezier curve that is populated with these segments. For torque calculations at each time step, we include a grasp force to reduce the chance of the robot dropping the cube. 

## Extra

### Extensions: 
After finishing the trajectory, control will check if the cube is within a threshold (EPSILON) of the target location, and if not will attempt to shift the cube to the target location. At this time it only works if the robot is still holding the cube and the cube is on the correct side of the obstacle.