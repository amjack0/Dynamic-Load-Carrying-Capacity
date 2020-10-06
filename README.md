# &#x1F539; Getting started 

This repository is developed to generate a trajectory using KDL, then optimization is performed on trajectory using NLOPT library.

# &#x1F539; Libraries required

1) NLOPT
2) KDL

Please refer this for installing the NLOPT ``` https://nlopt.readthedocs.io/en/latest/NLopt_Installation/ ```

# &#x1F539; About this repo 

Energy optimization, with Trajectory duration, target velocity, target acceleration and joint torques as constraints.


I calculate trajectory using KDL by providing two Cartesian points. Then from trajectory I read KDL::Frame of the end-effector in <br/> Cartesian space and I use IK solver to calculate joint angles. In a similar manner using IK solver joint velocity is calculated. <br/> Then using joint velocity , joint acceleration is deduced. Now, using Inverse dynamics solver with calculated joint angles,<br/> joint velocity and joint acceleration , torque is calculated and with that torque, energy is calculated as, <br/>
 ``` Energy = tau_.transpose() * tau_ ```

Constraint is given on joint torque which I take from motor torque (from urdf) for UR5. In this optimization, individual joint torques <br/>
are evaluated at each optimization step. This works same for Trajectory duration, target acceleration and velocity of the trajectory.



