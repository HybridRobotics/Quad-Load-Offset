

## Algorithm

- The approximate differential flatness is implemented by introducing two global variables 
<img src="http://www.forkosh.com/mathtex.cgi? \Large x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}">
. These two variables are updated in the controller `odefun_control2.m`.
```
R_ = R;
Omega_ = Omega;
```

- `odefun_control1.m`: Apply the zero offset geometric controller to the offset dynamical model, where we can observe a final configuration error.

- `odefun_control2.m`: Apply the geometric control to the offset dynamical model

