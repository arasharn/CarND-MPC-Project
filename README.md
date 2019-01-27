# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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

## Defining Car Model

The model of the cars is defined as:

$$x[t] = x[t-1] + v[t-1]cos(psi[t-1])dt$$
$$y[t] = y[t-1] + v[t-1]sin(psi[t-1])dt$$
$$\psi[t] = \psi[t-1] + \frac{v[t-1]}{Lf} \Delta[t-1]dt$$
$$v[t] = v[t-1] + a[t-1] dt$$
$$cte[t] = f(x[t-1]) - y[t-1] + v[t-1] sin(\epsilon[t-1]) * dt$$
$$\epsilon[t] = \psi[t] - \psi[t-1] + \frac{v[t-1] \delta[t-1]}{Lf}dt$$

Where:

$x$ and $y$ are position of the vehicle
$\psi$ is the heading orientation
$v$ is the velocity
$cte$ is the cross-track error that is distance of the vehicle's center line from the middle of the lane
$\epsilon$ is the orientation error

## main.cpp

main.cpp is initialized with bottom values from the simulator

* ptsx is the x-position of waypoints ahead on the track in global coordinates
* ptsy is the y-position of waypoints ahead on the track in global coordinates
* px is the current x-position of the vehicle's position in global coordinates
* py is the current y-position of the vehicle's position in global coordinates
* psi is the current orientation angle of the vehicle
* v is the current velocity of the vehicle
* steering_angle is the current steering angle
* throttle is the current throttle

After loading all the values, it is needed to transform them into the into the coordinate system of the vehicle by using the transformation matrix. In this code the waypoint vectors ptsx and ptsy are transformed to the vehicle's coordinate system and new waypoint vectors are called x_vehicle and y_vehicle. These two vectors specify the way the car is intended to follow.

In the next step, a third order polynomial is fitted to these waypoints using the `polyfit` function and the The cross-track error (cte) is then calculated by evaluating the polynomial function at the current position of the car using `polyeval`. It should be noticed since the path is transformed to the car coordinate system, the position and orientation of the car (px, py and psi) are all zeros in the car coordinate system. 

Also, the Latency of 100 ms for next step was added to predict the state of the system. This predicted state is then input to the controller.

Finally, states of the system and the polynomial coefficients aare needed to the MPC controller and the by using them controller estimates the steering angle and the throttle values to keep sth car in the track(it uses the `mpc.solve`).

## MPC.cpp

In general the goal of using the controller is to minimize the cost function for different factors such as:

* Sum of square values of cte and epsi to minimize cross-track and orientation errors.
* Sum of square values of (v - v_ref) to minimize the difference of the speed with the reference speed.
* Sum of square of actuator values a and delta to penalize large actuator actions.
* Sum of square values of the difference between two consecutive actuator values to penalize sharp changes.

To do that new weight parameters were defined to prioritize the importance of each factor in the cost function. The values for these weight values were obtained through trial and error that based on the observations weights corresponding to the steering angle input and its rate have the most significant impact on the performance of the system. Large values for these two weights (W_DELTA and W_DDELTA) help improving the stability of the car and avoiding the erratic and sudden steering behavior. 

## Timestep Length and Elapsed Duration (N & dt):

The number `N` and `dt` define the prediction horizon. Choosing a long prediction horizon can theoretically improve the prediction, but in practice, it increases the computational complexity. Due to the high number of points, the controller becomes slower and can become unstable. After trial and error it was found out that N = 10 and dt = 0.1 which corresponds to a 1-second time horizon work best for the model.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
