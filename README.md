# Robot-arm-writes-word

A UR5e robotic arm model was created in MATLAB and used to write the word "FLOW". A quintic spline interpolation method was implemented to generate the end-effect trajectories.

There are two '*.m' files in this directory with two methods to write 'FLOW'.

The 'functions' file is important, which contains code to implement various functions, should be added into the path before running the code in MATLAB. The relevant operation is already in the code (addpath('functions');).

---
* Method 1

The method 1 is used in 'main1.m'. The implementation details are as following:
(1) Set the way points.
(2) Choose two points in order, and add interpolation points as via points.
(3) In joint space, compute quintic spline interpolation.

How to execute the codes:
1. Open 'mian1.m' in MATLAB, click 'Run'.

---
* Method 2

The method 2 is used in 'main2.m'. The main idea is x_dot = J*q_dot. We know the path of the end effector, if we also know the cost time of each letter, we can get the average velocity 'x_dot'. Then we can apply the following formulas:

(1) q_dot = J^T(JJ^T)^(-1)*x_dot;
(2) q = q0 + q_dot;

The problem is that there is a cumulative error, if we can add PID controller for feedback control, the effect should be better.

How to execute the codes:
1. Open 'mian2.m' in MATLAB, click 'Run'.
