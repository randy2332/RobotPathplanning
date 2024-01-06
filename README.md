# RobotPathplanning

We plan a path for both **joint move** and  **cartesian move**

The starting, via, and end points for the path are

A = [0.64  0.77   0   0.05;
        0.77  -0.64   0   -0.55;
          0      0      -1   -0.6;
          0      0     0    1;];

B = [0.87  -0.1   0.48   0.5;
        0.29  0.9   -0.34   -0.4;
       -0.4  0.43   0.81   0.4;
         0     0          0    1;];

C = [0.41 -0.29   0.87  0.6;
        0.69  0.71   -0.09  0.15;
       -0.6  0.64   0.49  -0.3;
         0     0           0    1;];

The length unit above is cm.

The time to move from point A to B is 0.5 sec., and from point B to C 0.5 sec. The tacc for the transition portion is 0.2 sec. and the sampling time 0.002 sec.

For each move,we do the planning for both  straight line portion and  transition portion.

We also compute angle, velocity and acceleration of each.

### Execution

run main.m

### Result

a) **joint move**

![3DPathofJointSpacePlanning.png](https://github.com/randy2332/RobotPathplanning/blob/main/Pic/3DPathofJCartesianSpacePlanning.png)

b)**cartesian move**

![3DPathofJCartesianSpacePlanning.png](https://github.com/randy2332/RobotPathplanning/blob/main/Pic/3DPathofJointSpacePlanning.png)
