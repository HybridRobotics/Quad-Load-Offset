## Algorithm

- The approximate differential flatness is implemented by introducing two global variables. These two variables are updated in the controller `odefun_control2.m`.
```
R_ = R;
Omega_ = Omega;
```

- `odefun_control1.m`: Apply the zero offset geometric controller to the offset dynamical model, where we can observe a final configuration error.

- `odefun_control2.m`: Apply the geometric control to the offset dynamical model

