**PID Controller**

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

<<<<<<< HEAD
## Reflection
=======
##Reflection
>>>>>>> 4b60a5da1dcbb98c64d947ea50f8afdf3cc96250

### Describe the effect each of the P, I, D components had in your implementation.

#### Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected?
Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

<<<<<<< HEAD
In my implementation, the effect of the P component is reflecting the current error.I used it for the PID control of steering and throttle.

The effect of the I component is taking into account the continuous factor, e.g. past errors. I used it for the PID control only.

The effect of the D component is utilizing the latest change, the gap between the previous error and the current error. I used it for the PID control only.

[Steering]

Steering value = - Kp * p_error -Kd * d_error -Ki * i_error

Kp: 0.16
Ki: 0.0001
Kd: 3

[Throttle]

Throttle value = 1 -Kp * p_error -Kd * d_error -Ki * i_error

Kp: 0.8
Ki: 0.0
Kd: 3

=======
...
>>>>>>> 4b60a5da1dcbb98c64d947ea50f8afdf3cc96250

### Describe how the final hyperparameters were chosen.

#### Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination!

<<<<<<< HEAD
I implemented twiddle logic though, the final hyperparameters were chosen monually, because the twiddle implementation didn't work effectively in the period that I tried.
(Meanwhile, it was somehow helpful that I learned the concept throughout programming).
=======
...
>>>>>>> 4b60a5da1dcbb98c64d947ea50f8afdf3cc96250
