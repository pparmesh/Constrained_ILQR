# CILQR
Implements a Constrained Iterative LQR controller for an Autonomous Vehicle.<br/>
Implemented an iterative Linear
Quadratic Regulator (iLQR) algorithm that incorporates constraints in the environment for on-road autonomous motion
planning. Since iLQR is based on the theory of dynamic
programming, it does not inherently take constraints like
obstacles, actuator limits, etc into account. Therefore a Constrained Iterative Linear Quadratic Regulator [1] [2] (CILQR)
algorithm is used, which solves constrained optimal control
problem in nonlinear systems efficiently. The algorithm is then
deployed in an autonomous driving simulator, which will also
be used for validation of the project. <br/>

Here are the results in a self-developed Python Simulator <br/>
Two different behaviors are shown depending on the cost of deviating from the reference
trajectory and deviating from the desired speed. <br/>
In this first GIF, there is a high cost for deviating from the reference trajectory and hence the ego-vehicle(in red)
stays close to the reference path (red line) and does not overtake the NPC vehicle (in green)
![](https://media.giphy.com/media/S3ar1cwuQ5V59uyt65/giphy.gif) <br/>
In this second GIF, there is a high cost for deviating from the desired velocity and hence the ego-vehicle( in red)
deviates from the reference path (red line), maintains its desired speed and overtakes the NPC vehicle (in green)
![](https://media.giphy.com/media/j3nDL8cGCu0T7NZeO1/giphy.gif) <br/>
References <br/>
1. Jianyu Chen, Wei Zhan, and Masayoshi Tomizuka. Constrained iterative
lqr for on-road autonomous driving motion planning. In 2017 IEEE 20th
International Conference on Intelligent Transportation Systems (ITSC),
pages 1–7. IEEE, 2017.
2. Jianyu Chen, Wei Zhan, and Masayoshi Tomizuka. Autonomous driving
motion planning with constrained iterative lqr. IEEE Transactions on
Intelligent Vehicles, 4(2):244–254, 2019. <br/>

Contributors: Prateek Parmeshwar, Karmesh Yadav
