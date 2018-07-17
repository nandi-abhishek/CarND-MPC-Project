
# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

This project implemented a Model Predictive Controller to drive a car in Udacity provided simulator around a track. There are few sharp turns in the track which is difficult to maneuver. The MPC engine recevies car telemetry information using [WebSocket](https://en.wikipedia.org/wiki/WebSocket). And it feeds the simulator with steering angle and throttle. There is 100 ms latency between generation of a pair of data and its application. The solution should be able to handle it robustly.

The solution uses couple of libraries - IPOPT and CPPAD for calculation as suggested in the course. The implementation is pretty straight forward as it was mostly similar to the lessons taught but tuning the parameter is main trick.


## Rubric Points

### The Model**: 

The model implemented is the simpler Kinematic model which performs well in most of the cases at low speed. I dont think there is scope of using any other dynamic model as no other details related to tire force etc are given. The model equations are given below:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:

- `x, y` : Car's position.
- `psi` : Car's heading direction.
- `v` : Car's velocity.
- `cte` : Cross-track error.
- `epsi` : Orientation error.

They comprises the state of the model. In addition to that, `Lf` is the distance between the car of center of gravity and the front wheels  and this is provided in the seed project directory. The other two values are the model output:

- `a` : Car's acceleration (throttle).
- `delta` : Steering angle.

The objective is to find the acceleration (`a`) and the steering angle(`delta`) in the way it will minimize an objective function that is the combination of different factors:

- Square sum of `cte` and `epsi`. It could be found [here](./src/MPC.cpp#L51).
- Square sum of the difference actuators to penalize a lot of actuator's actions. It could be found [here](./src/MPC.cpp#L58).
- Square sum of the difference between two consecutive actuator values to penalize sharp changes. It could be found [here](./src/MPC.cpp#L65).

How much weight each of these factors had were tuned manually to obtain a successful track ride without leaving the road. I tried with 1000 as the mutiplication factor for 'cte' and 'epsi'. But car was going off the track with these values so I increases the weightage for them.
The other parameter that helped a lot is tuning of the change of actuators. Increasing the value made the car trajectory smoother.

### Timestep Length and Elapsed Duration (N & dt)**: 

The values chosen for N and dt are 10 and 0.1, respectively. The values are obtained from Udacity's video lecture on the project. We learnt how changing the values would effect the behavior in the course and also in that video. A smaller dt values means we are recalculating the position of the car frequently with higher resolution. 

As far as N is concerned if we increase N then it becomes difficult for solver to fit a solution. This would become computationally expensive. I have tried increasing N to 20 and 25 but even car was not able to complete a full lap with these values at reference speed 100.

After keeping N to 10 I started adjusting dt. I was able to achieve speed of 93 mph with dt set to 0.08. But the car was slightly going into the red curb so, finally settled on the values 10 and 0.1 and achieved max spped of 91 mph. 

### Polynomial Fitting and MPC Preprocessing**: 

The simulator gives MPC few waypoints but they are in MAP co-ordinate to I have used simple co-ordinate transformation (see main.cpp lines 101-106) to change them to vehicle's perspective. This simplifies fitting a polynomial because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero. This was also suggested in the official video for the project.

### Model Predictive Control with Latency**: 

The other key trick is to handle actuator latency. So, instead of setting the state values to the simulator data, I have added a delay interval of (100 ms) and calculated future state based on the model equation. Then I have obtained the new state which is sent to solver. The code implementing that could be found at [./src/main.cpp](./src/main.cpp#L127) from line 127 to line 137.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Simulation

### The vehicle must successfully drive a lap around the track.

The vehicle successfully drives a lap around the track. Here is a short video with the final parameters: [./video/MPC_car.mp4](./video/MPC_car.mp4).

