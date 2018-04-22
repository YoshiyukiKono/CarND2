**Model Predictive Control (MPC)**

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

## Implementation

### The Model

#### Student describes their model in detail. This includes the state, actuators and update equations.

[State & Actuators(latency = 0)]
* x : 0
* y : 0
* psi : 0;
* velocity : velocity
* cte : coeffs[1] + coeffs[2] x 2 + coeffs[3] x 3
* epsi : psi - atan(coeffs[1])

[Model Update Equation]

| Equation        | Remarks   | 
|:-------------:|:-------------|
| x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt | x position |
| y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt | y position |
| psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt | orientation |
| v_[t] = v[t-1] + a[t-1] * dt | velocity |
| cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt | the current cross track error plus the change in error caused by the vehicle's movement |
| error_psi[t] = psi[t] - desired_psi[t-1] + v[t-1] * delta[t-1] / Lf * dt | the desired orientation subtracted from the current orientation |


| Equation        | Remarks   | 
|:-------------:|:-------------|


* f(x[t-1]): coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3]*x0*x0*x0
* desired_psi[t-1]: atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1])

### Timestep Length and Elapsed Duration (N & dt)

#### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

I chose these values: N = 10 & dt = 0.1

Neither bigger nor smaller N did not work as follows.

*  N = 20 & dt = 0.1: The car cannot run safely even when starting.
*  N =  5 & dt = 0.1: The car cannot run safely.

Neither bigger nor smaller dt also did not work as follows.

*  dt = 0.01 & N = 10: The car cannot run on the road even just after starting.
*  dt = 1    & N = 10: The car was running verry slowly at first, then stoped before the bridge.

### Polynomial Fitting and MPC Preprocessing
#### A polynomial is fitted to waypoints.If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

[Waypoints Preprocess]
*  ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi))
*  ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi))
Shift X/Y is the distnace between each point and the car.

[Vehicle State & Actuators]

See below.

### Model Predictive Control with Latency
#### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

I dealted with the latency as described below.

[State & Actuators (with latency: time t)]
* x : velocity[t-1] x latency
* y : 0
* psi : velocity[t-1] x steer_value/Lf x latency;
* velocity : velocity[t-1] + throttle_value x latency
* cte : cte[t-1] + velocity[t] x sin(epsi[t]) xlatency
* epsi : psi[t] - atan(coeffs[1])
