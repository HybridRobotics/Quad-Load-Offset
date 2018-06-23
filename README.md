# Implementation

## Description of algorithm

- `odefun_control1.m`: Geometric control on SE(3)xR^2 quadrotor suspended with point mass. which illustrates the Matlab simulation and implementation of 'Geometric Control and Differential Flatness of a Quadrotor UAV with a Cable-Suspended Load - Koushil Sreenath, Taeyoung Lee, Vijay Kumar' link https://ieeexplore.ieee.org/document/6760219/

- `odefun_control2.m`: Apply previous geometric control on SE(3)xR^2 quadrotor suspended with point mass into the offset dynamical model, where we can observe a final configuration error.

- `odefun_control3.m`: Apply improved geometric control on SE(3)xR^2 quadrotor suspended with point mass where there is an offset from the suspension point to the CoM of quadrotor. The desired states are generated by the differential flatness.

- `odefun_control4.m`: Apply impproved geometric control on SE(4)xR^2 quadrotor suspended with point mass where there is an offset from the suspension point to the CoM of quadrotor. The desired states are generated by the differential flatness.




