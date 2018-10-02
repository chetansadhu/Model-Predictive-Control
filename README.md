# **Model Predictive Control** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Compiling
---
The project is compiled in Ubunutu 18.04.
To compile,
```
    mkdir build
    cd build
    cmake ..
    make
    ./mpc
 ```

Implementation
---
### Model
Model includes 6 states and 2 actuators along with the cost.
6 state vectors are: `[x, y, psi, v, cte, epsi]`
2 actuators are: `[delta, a]` (steering angle and throttle).

Update equations:
```
    x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi[t+1] = psi[t] + v[t] * delta[t] / Lf * dt
    v[t+1] = v[t] + a[t] * dt
    cte[t+1] = cte[t] + f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    espi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
 ```
With the update equations fed to the `ipopt solver`, it solves for the actuators, `delta` and `a`.
The initializations, upper bound and lower bound for the variables and constraints are taken from the classroom lesson code.

### Timestep length and Elapsed duration
| N | dt | Result |
|---------------:|:----------:|:-------------:|
| 5 | 0.1 | The timestep length isn't enough for the car to make predictions and to turn. |
| 10 | 0.1 | This seemed very smooth and was able to complete the track successfully. |
| 15 | 0.1 | Initially was smooth but lost some stability in turns, although it corrected in next timestep. |
| 10 | 0.2 | With increased dt car became slow during turns and at times the difference between the actual waypoints and predicted way points is more and car touches the line during turns |

I am finally stuck with `N = 10` and `dt = 0.1` as it produced good performance.

### Polynomial fitting
The points `ptsx` and `ptsy` are first transformed from global coordinates to car coordinates. These transformed points are used in a `third degree polyfit` to fit a cubic curve along which car is beleived to traverse.
Since the fitting results are in car coordinates, the state vector x, y, psi becomes 0 and `cte = coeffs[0]` and `epsi = -arctan(coeffs[1])`

### Latency
There is a 100ms (0.1s) latency with car responding to the actuators. To account for this latency the state vector given to the model is predicted with dt = 0.1 (the 100ms latency). Instead of car responding to the previous actuation now with the predicted state car responds to the current actuation.