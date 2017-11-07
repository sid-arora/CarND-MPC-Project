# CarND-Controls-MPC

[image1]: ./output_img/1.png "Output Image 1"
[image2]: ./output_img/2.png "Output Image 2"
[image3]: ./output_img/3.png "Output Image 3"
[image4]: ./output_img/4.png "Output Image 4"

The project is the final project of the second term in the Udacity Self-Driving Car nanodegree program. The aim is to implement Model Predictive Controller to drive a car around the track to follow an expected trajectory using the Udacity [Simulator](https://github.com/udacity/self-driving-car-sim/releases). I followed the Global Kinematic Model and tuned the parameters in order to reach maximal speed.

Here is the **result** :
[YouTube video](https://youtu.be/SgfP8kDn5-I)

The Global Kinematic model can be distinguished into three parts: *State*, *Actuators*, *Equations*.

The vehicle's state consists of the following elements:

* (x,y): x,y co-ordinates of vehicle
* Ψ: Orientation Angle
* v: speed
* cte: Cross Track Error
* epsi: Orientation Error

The actuators include :
* Steering : Steering angles span the interval of [-25,+25] degrees 
* Throttle : Throttle works the same way within the interval of [-1,+1], where -/+ represent acceleration/deceleration

The equations used tp update hte model state include:

* x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(Ψ<sub>t</sub>) * dt
* y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(Ψ<sub>t</sub>) * dt
* Ψ<sub>t+1</sub> = Ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * steer * dt
* v<sub>t+1</sub> = v<sub>t</sub> * throttle * dt

where Lf measures the distance between the front of the vehicle and its center of gravity and the parameter was provided by Udacity.

Errors: Cross Track Error (cte) and ψ error (eψ) were used to build the cost function for the MPC. They could be updated on a new time step using the following equations:
* cte<sub>t+1</sub> = cte<sub>t</sub> + v<sub>t</sub> * sin(epsi<sub>t</sub>) * dt
* epsi<sub>t+1</sub> = epsi<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * steer * dt

## MPC Solution

This project mainly comprised of tuning parameters of the cost function for the Model Predictive Controller. 
The Model Predictive Control has been implemented using the Ipopt, which is a library for large-scale nonlinear optimization.

#### The code

The solution is significantly based on the 
[implementation of the MPC provided in the Udacity lessons](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp).

#### Polynomial Fitting and MPC Preprocessing

Steps taken:
* Waypoints were was transformed into the vehicle space 
* A latency of 100ms was added to shift the state of the vehicle into the future increasing stability of the controller and simulating real human delay 
* Cross track error and orientation error were calculated
* Data was passed into the MPC routine.

The cost function parameters were chosen by trial and error. They were tuned in order to reach maximal speed by using the whole width of the road and breaking before turns. These parameters are stored in the `src/MPC.h` file:

```
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set cte, epsi and speed
// Ref cross track and orientation errors = 0
const double ref_cte = 0.0;
const double ref_epsi = 0.0;

// Reference Velocity
const double ref_v = 81.5;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;
```
#### Timestep Length and Elapsed Duration (N & dt)
T, the predictive horizon interval, is the product of two other variables, N and dt. N is the number of timesteps in the horizon and dt is the time elapsed between actuations. It was shown that the MPC could drive safely around the track with _T_ = 1 s, but on a slow speed. Higher speed requires more future information to make smart decisions during turns. Time step duration (_dt_) was chosen as 0.08s and 8 time steps (_N_) was used. These values were chosen to decrease computation time while going at a higher speed while staying on the track. 

#### Latency 

Using the model state equations, the delayed state was calculated and fed to the model:
```
double latency = 0.1; 
px = v * latency;
py = 0.0;
psi = -v * steer / Lf * latency;
cte = cte + (v * sin(epsi) * latency);
epsi = epsi - v * steer / Lf * latency;
v = v + throttle * latency;
state << px, py, psi, v, cte, epsi;
```
* Reference velocity (v<sub>ref</sub>)  = 81.5

Increasing the value for reference velocity led to increase in the constants responsible for restricting steering angle. 

Latency was handled by adding a delay of 100 ms to account for actuations from the previous step and by penalizing rate of change in actuation ( Lines 29-39 in MPC.cpp ) enabled better cornering.

## Hardware 

The performance and quality of the controller strongly depended on computational power of my hardware. Weak hardware caused latency.

* The project was developed and run on an 2.4 GhZ Intel Core I5 with 16 GB of RAM. 
* Simulator was setted to use 640x480 px resolution and the "Simple" graphics. The controller was prone to abnormal behavior with higher resolution.

## Files altered

- `src/MPC.h` - MPC parameters
- `src/main.cpp` - includes communication with the Simulator and data preprocessing
- `src/MPC.cpp` - MPC implementation


## OUTPUT IMAGES

![alt text][image4]

![alt text][image2]

![alt text][image3]

![alt text][image1]

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
