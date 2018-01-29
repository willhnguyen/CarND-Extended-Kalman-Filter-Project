# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project, students are asked to implement the extended Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

## Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build and Run Instructions
To build the program, run the following sequence of command line instructions.

```bash
$ cd /path/to/cloned/repo
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To see the program in action, make sure to download the [Term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases), run it, and choose the first simulation option. Then, execute the program by executing the `ExtendedKF` program in the `build` folder.

```bash
$ ./ExtendedKF
```

## Project Information
### Kalman Filter
The Kalman filter is an algorithm that predicrts an object's movement by using sensor data. Because sensor data is noisy and has a degree of uncertainty, the Kalman filter is used to provide a more accurate prediction of the object's actual movement. It does so by looping through a prediction step and a measurement update step.

The Kalman filter keeps an updated state vector for the object which includes the object's cartesian position and velocity.

![Kalman state vector](https://latex.codecogs.com/gif.latex?x%20%3D%20%5Cbegin%7Bbmatrix%7D%20p_x%5C%5C%20p_y%5C%5C%20v_x%5C%5C%20v_y%20%5Cend%7Bbmatrix%7D)

#### Prediction Step
The prediction step predicts the object's position after some amount of time. It does so by using the current known state of the object.

![Prediction Step Equation](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7D%20%3D%20F%20%5Ccdot%20x_t%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%20%5CDelta%20t%20%26%200%20%5C%5C%200%20%26%201%20%26%200%20%26%20%5CDelta%20t%20%5C%5C%200%20%26%200%20%26%201%20%26%200%20%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7D%20p_x%5C%5C%20p_y%5C%5C%20v_x%5C%5C%20v_y%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20p_x%20&plus;%20v_x%5CDelta%20t%5C%5C%20p_y%20&plus;%20v_y%5CDelta%20t%5C%5C%20v_x%5C%5C%20v_y%20%5Cend%7Bbmatrix%7D)

The prediction step also updates the process noise which represents the noise of the system over time.
 
![Process Noise Update Equation](https://latex.codecogs.com/gif.latex?P%20%3D%20F%20%5Ccdot%20P%20%5Ccdot%20F%5ET%20&plus;%20Q)

[nu]: https://latex.codecogs.com/gif.latex?%5Cnu

The Q term is derived from the state uncertainty term ![nu][nu]. For this README, the derivation of the ![nu][nu] and Q terms will not be included for brevity.

![Q Term](https://latex.codecogs.com/gif.latex?Q%20%3D%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B4%7D%20%5Csigma_%7Bax%7D%5E2%20%26%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%5Csigma_%7Bax%7D%5E2%20%26%200%5C%5C%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E4%7D%7B4%7D%20%5Csigma_%7Bay%7D%5E2%20%26%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%5Csigma_%7Bay%7D%5E2%5C%5C%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%5Csigma_%7Bax%7D%5E2%20%26%200%20%26%20%5CDelta%20t%5E2%20%5Csigma_%7Bax%7D%5E2%20%26%200%5C%5C%200%20%26%20%5Cfrac%7B%5CDelta%20t%5E3%7D%7B2%7D%20%5Csigma_%7Bay%7D%5E2%20%26%200%20%26%20%5CDelta%20t%5E2%20%5Csigma_%7Bay%7D%5E2%20%5Cend%7Bbmatrix%7D)

#### Measurement Update Step
Once the object's state has been predicted, it is necessary to validate the prediction with sensor data. There are 5 equations for the measurement update step.

![y equation](https://latex.codecogs.com/gif.latex?y%20%3D%20z%20-%20H%20%5Ccdot%20x)

![S equation](https://latex.codecogs.com/gif.latex?S%20%3D%20H%20%5Ccdot%20P%20%5Ccdot%20H%5ET%20&plus;%20R)

![K equation](https://latex.codecogs.com/gif.latex?K%20%3D%20P%20%5Ccdot%20H%5ET%20%5Ccdot%20S%5E%7B-1%7D)

![x equation](https://latex.codecogs.com/gif.latex?x%27%3D%20x%20&plus;%20K%20%5Ccdot%20y)

![P equation](https://latex.codecogs.com/gif.latex?P%27%20%3D%20%28I%20-%20K%20%5Ccdot%20H%29%20%5Ccdot%20P)

The first equation calculates the difference between the predicted state and the measured state. The H vector is required to convert the state vector x into the same state space of the sensor's measurement data. For LiDAR, the H data is provided below. Read the Extended Kalman filter section for RADAR.

![H matrix](https://latex.codecogs.com/gif.latex?H%20%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%200%20%26%200%5C%5C%200%20%26%201%20%26%200%20%26%200%20%5Cend%7Bbmatrix%7D)

The R matrix represents the sensor noise. This matrix is a diagonal matrix that has uncertainty measurements provided by the manufacturer of the sensor.

The S and K matrices represent the covariance matrix  for y and the Kalman gain matrix. These are calculated to best predict the object's actual state.

### Extended Kalman Filter
The extended Kalman filter is an extension of the Kalman filter that allows for non-linearity. This non-linearity occurs when using RADAR since the y equation in the measurement update step has to use square roots and trigonometric functions to convert the cartesian state space to RADAR's polar state space.

![RADAR y equation](https://latex.codecogs.com/gif.latex?y%20%3D%20z%20-%20h%28x%29%20%3D%20z%20-%20%5Cbegin%7Bbmatrix%7D%20%5Csqrt%7Bp_x%5E2&plus;p_y%5E2%7D%5C%5C%20%5Ctext%7Barctan%7D%28p_y/p_x%29%5C%5C%20%5Cfrac%7Bp_x%20v_x%20&plus;%20p_y%20v_y%7D%7B%5Csqrt%7Bp_x%5E2&plus;p_y%5E2%7D%7D%20%5Cend%7Bbmatrix%7D)

As a result, RADAR has no equivalent H matrix to calculate the S and K matrices. To accomodate for this non-linearity, a Jacobian matrix (the first term of a multidimensional Taylor expansion) can be used to approximate an equivalent H matrix. Again, for this README, the Jacobian matrix derivation will not be included for brevity.

![Radar Jacobian Matrix](https://latex.codecogs.com/gif.latex?H_j%20%3D%20%5Cbegin%7Bbmatrix%7D%20%5Cfrac%7Bp_x%7D%7B%5Csqrt%7Bp_x%5E2&plus;p_y%5E2%7D%7D%20%26%20%5Cfrac%7Bp_y%7D%7B%5Csqrt%7Bp_x%5E2&plus;p_y%5E2%7D%7D%20%26%200%20%26%200%5C%5C%20-%5Cfrac%7Bp_y%7D%7Bp_x%5E2&plus;p_y%5E2%7D%20%26%20%5Cfrac%7Bp_x%7D%7Bp_x%5E2&plus;p_y%5E2%7D%20%26%200%20%26%200%5C%5C%20%5Cfrac%7Bp_y%28v_x%20p_y%20-%20v_y%20p_x%29%7D%7B%28p_x%5E2&plus;p_y%5E2%29%5E%7B3/2%7D%7D%20%26%20%5Cfrac%7Bp_x%28v_y%20p_x%20-%20v_x%20p_y%29%7D%7B%28p_x%5E2&plus;p_y%5E2%29%5E%7B3/2%7D%7D%20%26%20%5Cfrac%7Bp_x%7D%7B%5Csqrt%7Bp_x%5E2&plus;p_y%5E2%7D%7D%20%26%20%5Cfrac%7Bp_y%7D%7B%5Csqrt%7Bp_x%5E2&plus;p_y%5E2%7D%7D%20%5Cend%7Bbmatrix%7D)

## Project Implemenation Details
Following the provided starter code, the `src/FusionEKF.cpp`, `src/kalman_filter.cpp`, and `src/tools.cpp` files were modified as necessary. For this project, `src/kalman_filter.h` was also modified to add an extra class method to avoid repeating the measurement update calculations that were the same for both RADAR and LiDAR.

The updated `src/FusionEKF.cpp` file includes variable initialization and code that controls how RADAR and LiDAR data are processed.

The updated `src/kalman_filter.cpp` file has the entire Kalman filter and extended Kalman filter implemented. This includes the predictions step and the measurement update steps for both LiDAR and RADAR.

The updated `src/tools.cpp` file includes a root mean square error calculator and a Jacobian matrix generator.
