# CILQR
Implements a Constrained Iterative LQR controller for an Autonomous Vehicle.<br/>
Implemented an iterative Linear
Quadratic Regulator (iLQR) algorithm that incorporates con-
straints in the environment for on-road autonomous motion
planning. Since iLQR is based on the theory of dynamic
programming, it does not inherently take constraints like
obstacles, actuator limits, etc into account. Therefore a Con-
strained Iterative Linear Quadratic Regulator [1] [2] (CILQR)
algorithm is used, which solves constrained optimal control
problem in nonlinear systems efficiently. The algorithm is then
deployed in an autonomous driving simulator, which will also
be used for validation of the project. 
1. owdj
Here are the results in a self-developed Python Simulator <br/>
![](https://media.giphy.com/media/S3ar1cwuQ5V59uyt65/giphy.gif)
