
# Extended Kalman Filter

  

## Project goal

  

The goal of this project is to track the bicycle's position and velocity by using a Kalman filter, lidar measurements and radar measurements. The EKF algorithm is run against the dataset located in "data/obj_pose-laser-radar-synthetic-input.txt". This dataset corresponds to simulated lidar and radar measurements detecting a bicycle that travels around the vehicle.

## Running the program

After cloning the repo:
- run the script install_ubuntu.sh to install the necessary modules like:
	- cmake: 3.5
	- make: 4.1 (Linux and Mac)
- gcc/g++: 5.4
- [uWebSocketIO]([https://github.com/uWebSockets/uWebSockets](https://github.com/uWebSockets/uWebSockets)) library.
- Install the [Udacity Simulator]([https://github.com/udacity/self-driving-car-sim](https://github.com/udacity/self-driving-car-sim))
- Build the program from this repo using the following commands:

>

  

    mkdir build
    cd build
    cmake ..
    make

- Run the program by executing `./ExtendedKF`
- Launch the simulator in parallel to see the tracking of the bicycle.

  

## Source files

The source files are located in the `src` folder:

  

- `main.cpp` - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE

- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function

- `kalman_filter.cpp`- defines the predict function, the update function for lidar, and the update function for radar

- `tools.cpp`- function to calculate RMSE and the Jacobian matrix

 ## Overview of Kalman Filter

The three main steps for programming a Kalman filter are:

  

- **initializing** Kalman filter variables

- **predicting** where our object is going to be after a time step \Delta{t}Δt

- **updating** where our object is based on sensor measurements

  

Then the prediction and update steps repeat themselves in a loop.

  

To measure how well the Kalman filter performs, the **root mean squared error** is calculated by comparing the Kalman filter results with the provided ground truth.

  

## Process measurement

  

The workflow for the process measurement (based on Lidar and Radar measurements) used for implementing the EKF algorithm is described by the picture below:

![enter image description here](https://raw.githubusercontent.com/vincentbarrault/Extended-Kalman-Filter/master/img/Process%20measurement.png)
 

  

## Result

![enter image description here](https://raw.githubusercontent.com/vincentbarrault/Extended-Kalman-Filter/master/img/Video_large.gif)
