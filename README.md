# Quad-Load-Offset
A quadrotor with a point-mass payload suspended with an offset from the center-of-mass of the quadrotor to the suspension point is studied in this paper. A comparison of a geometric controller developed for a zero offset quadrotor-Ioad model is presented to motivate the need as well as demonstrate the advantages of our proposed geometric controller for the offset quadrotor-Ioad model.

## References
If you find this project useful in your work, please consider citing following paper:
J. Zeng and K. Sreenath. "Geometric Control of a Quadrotor with a Load Suspended from an Offset." *2019 American Control Conference (ACC)*. [[IEEE]](https://ieeexplore.ieee.org/document/8814939) 
```
@inproceedings{zeng2019offset,
  author={Zeng, Jun and Sreenath, Koushil},
  booktitle={2019 American Control Conference (ACC)}, 
  title={Geometric Control of a Quadrotor with a Load Suspended from an Offset}, 
  year={2019},
  volume={},
  number={},
  pages={3044-3050},
}
```
## Algorithm
- The approximate differential flatness is implemented by introducing two global variables. These two variables are updated in the controller `odefun_control2.m`.
```
R_ = R;
Omega_ = Omega;
```

- `odefun_control1.m`: Apply the zero offset geometric controller to the offset dynamical model, where we can observe a final configuration error.
- `odefun_control2.m`: Apply the geometric control to the offset dynamical model

## Results
The controller performance can be found in two folders `non_offset_control` and `offset_control` and it turns out that the proposed controller is necessary when there is an offset between the suspension point and the center of the quadrotor.