clc; clear;
close all;
%We have to change unit from centermeter to meter
A = [0.64  0.77   0   0.05;
    0.77  -0.64   0   -0.55;
     0      0    -1   -0.6;
     0      0     0    1;];
 
B = [0.87  -0.1   0.48   0.5;
     0.29  0.9   -0.34   -0.4;
     -0.4  0.43   0.81   0.4; 
      0     0     0    1;];
 
C = [0.41 -0.29   0.87  0.6;
     0.69  0.71   -0.09  0.15;
    -0.6  0.64   0.49  -0.3;
     0     0       0    1;];
 
% setup  sample rate
SamplingTime = 0.002;

%joint part
[q,dq,d2q] = JointMotion(A,B,C,SamplingTime);
plot_joint_motion(A,B,C,SamplingTime,q,dq,d2q)

%cartesion part
[x,y,z,a] = CartesianMotion(A,B,C,SamplingTime);
plot_Cartesian_motion(A,B,C,SamplingTime,x,y,z,a);

