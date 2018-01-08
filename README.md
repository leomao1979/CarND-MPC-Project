# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[equations]: ./images/equations.png "Equations"


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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Rubric Points

[rubric points](https://review.udacity.com/#!/rubrics/896/view)

### Your code should compile
Code compiles with `cmake` and `make` without errors.

### MPC Model

The state of MPC model includes:

x: x coordinate of vehicle's current postion 
y: y coordinate of vehicle's current position
𝛹(psi): vehicle's current orientation angle
v: vehicle's current speed
cte: cross track error, the error between road center and the vehicle's position
e𝛹(epsi): orientation error, the desired orientation subtracted from current orientation

The actuators include steering angle "𝛿" (delta) and acceleration "a".

Below are the equations to determine next state from current state:

![Equations][equations]

### Timestep Length and Elapsed Duration (N & dt)

I tried different combinations of N & dt, include (N: 20, dt: 0.05), (N: 20, dt: 0.08) and (N: 10, dt: 0.1). After tuning the cost function parameters, I could get the car run at 90+ MPH with the last combination. So the selected timestep length is 10 and elapsed duration is 0.1s.

### Polynomial Fitting and MPC Preprocessing

The received waypoints are in map's coordinate system. To make it easier for CTE and Epsi calculation, as well as predicted trajectory display in simulator, they are transformed to vehicle's coordinate system before polynomial fitting.

3-degree polynominal is used for this project.

```
    // Transform to vehicle coordinate system
    vector<double> ptsx_vcs, ptsy_vcs;
    for (int i=0; i<ptsx.size(); i++) {
        double x_vcs = cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
        double y_vcs = cos(psi) * (ptsy[i] - py) - sin(psi) * (ptsx[i] - px);
        ptsx_vcs.push_back(x_vcs);
        ptsy_vcs.push_back(y_vcs);
    }

    // Fit a polynomial to the above x and y coordinates
    Eigen::VectorXd pts_xvals = Eigen::VectorXd::Map(ptsx_vcs.data(), ptsx_vcs.size());
    Eigen::VectorXd pts_yvals = Eigen::VectorXd::Map(ptsy_vcs.data(), ptsy_vcs.size());
    auto coeffs = polyfit(pts_xvals, pts_yvals, 3);

```

### Model Predictive Control with Latency

To deal with latency, the initial state is re-calculated with current state and duration of delay.

```
    double delta = -steering * deg2rad(25), a = throttle;
    // New initial state with latency
    double x_latency = px_vcs + v * cos(psi_vcs) * latency;
    double y_latency = py_vcs + v * sin(psi_vcs) * latency;
    double psi_latency = psi_vcs + v * delta / Lf * latency;
    double v_latency = v + a * latency;
    double cte_latency = cte + v * sin(epsi) * latency;
    double epsi_latency = epsi + v * delta / Lf * latency;

    Eigen::VectorXd state(6);
    state << x_latency, y_latency, psi_latency, v_latency, cte_latency, epsi_latency;

```


The predicted trajectory returned from MPC is in vehicle's coordinate system with respect to current state. However, with latency when it is displayed in simuatlor, the new origin would be different. So, the resulting trajectory should be transformed accordingly.

```
    // Display the MPC predicted trajectory (Green line)
    // Points are in reference to the vehicle's coordinate system with respect to new position after latency
    vector<double> mpc_x_vals;
    vector<double> mpc_y_vals;
    const int mpc_x_start = 2;
    for (int i = 0; i < 2 * (N - 2); i += 2) {
        double mpc_x = vars[mpc_x_start + i], mpc_y = vars[mpc_x_start + i + 1];
        double x_transform = cos(psi_latency) * (mpc_x - x_latency) + sin(psi_latency) * (mpc_y - y_latency);
        double y_transform = cos(psi_latency) * (mpc_y - y_latency) - sin(psi_latency) * (mpc_x - x_latency);
        if (x_transform > 0) {
            mpc_x_vals.push_back(x_transform);
            mpc_y_vals.push_back(y_transform);
        }
    }
    double max_mpc_x = *std::max_element(mpc_x_vals.begin(), mpc_x_vals.end());
    if (max_mpc_x < 8) {
        mpc_x_vals.clear();
        mpc_y_vals.clear();
    }
    msgJson["mpc_x"] = mpc_x_vals;
    msgJson["mpc_y"] = mpc_y_vals;

```


### The vehicle must successfully drive a lap around the track.
Yes, the vehicle will successfully drive around the track with the chosen parameters. Please see output/video.mp4 for details.






