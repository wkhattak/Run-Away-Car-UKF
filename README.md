# Project: Catching Runaway Car with Unscented Kalman Filter
## Overview  

This project is about utilizing a [Unscented Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter) to estimate the position of a runaway car with noisy LiDAR and Radar measurements & then using these estimates to move the chase car towards the runaway car. 


## Project Introduction

In this project, not only do we implement an UKF, but also use it to catch an escaped car driving in a circular path. 
The runaway car is sensed by a stationary sensor, that is able to measure both noisy LiDAR and Radar data. The capture vehicle will need to use these measurements to close in on the runaway car. To capture the runaway car the capture vehicle needs to come within `0.1` unit distance of its position. However the capture car and the runaway car have the same max velocity, so if the capture vehicle wants to catch the car, it will need to predict where the car will be ahead of time.

## How Does It Work?

To predict the position of the runaway car, a *constant turn rate and velocity magnitude (CTRV)* motion model is used. The simulator provides LiDAR and Radar measurements that are utilized by the Unscented Kalman Filter(UKF) to provide estimated position of the target (runaway) car.
The UKF works in two steps; *predict* and *update*. In the *predict* step, based on the time difference alone (between previous & current timestamps), a prediction is made, whereas in the *update* step, the belief in object's location is updated based on the received sensor measurements.

## Catching the Runaway Car

The trick in this project is to use the **Predict** part of the UKF to make a prediction in future, as just finding out the current location of the target car won't help to catch it as both cars have the same max velocity. So what needs to be done is to find where the target car would be in future so that we can position the chase car at that location.

As described above, the *Predict* step pf the UKF only makes use of the previous location of the target car & the required time delta (how far in future we need the prediction). So that's why it's important to only use the *Predict* step of the UKF. However, we still need the complete cycle of *Predict* & *Update* to be able to precisely know the actual (current) location of the target car for making successive future predictions & adjusting the steering angle accordingly.

The pertinent lines of code in `main.cpp` are as follows:

```c++

          // get calculated x,y based on current timestamp
          target_x = ukf.x_[0];
          target_y = ukf.x_[1]; 
          // find distance to target based on current timestamp
          double distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));
          
          double heading_difference = 0.0;
          // predict 0.45 seconds ahead if distance to target is greater than 0.7 units
          if (distance_difference > 0.7) {
            // save current matrices based on current timestamp
            // the x_ matrix contains x,y positions of the target based on current timestamp
            VectorXd current_x_ = ukf.x_;
            MatrixXd current_P_ = ukf.P_;    
            
            // predict 0.45 seconds ahead
            ukf.Prediction(0.45);
            VectorXd future_x_ = ukf.x_;
            MatrixXd future_P_ = ukf.P_;
            
            // restore current matrices
            ukf.x_ = current_x_;
            ukf.P_ = current_P_;
            
            target_x = future_x_[0];
            target_y = future_x_[1];      

            // find theta
            double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
            // normalize theta
            while (heading_to_target > M_PI) heading_to_target-=2.*M_PI; 
            while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
            
            // turn towards the target based on heading difference 
            heading_difference = heading_to_target - hunter_heading;
            // normalize the angle
            while (heading_difference > M_PI) heading_difference-=2.*M_PI; 
            while (heading_difference <-M_PI) heading_difference+=2.*M_PI;
          }
          else { // predict only 0.3 seconds ahead to smooth the chase car's movement & to make sure it doesn't stay ahead of the target
            // save current matrices based on current timestamp
            // the x_ matrix contains x,y positions of the target based on current timestamp
            VectorXd current_x_ = ukf.x_;
            MatrixXd current_P_ = ukf.P_;    
            
            // predict 0.3 seconds ahead
            ukf.Prediction(0.3);
            VectorXd future_x_ = ukf.x_;
            MatrixXd future_P_ = ukf.P_;
            
            // restore current matrices 
            ukf.x_ = current_x_;
            ukf.P_ = current_P_;
            
            target_x = future_x_[0];
            target_y = future_x_[1];      

            // find theta
            double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
            // normalize theta
            while (heading_to_target > M_PI) heading_to_target-=2.*M_PI; 
            while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
            
            // turn towards the target based on heading difference 
            heading_difference = heading_to_target - hunter_heading;
            // normalize the angle
            while (heading_difference > M_PI) heading_difference-=2.*M_PI; 
            while (heading_difference <-M_PI) heading_difference+=2.*M_PI;
          }

```

[Here's a video clip](./video/Run away car - UKF_1.mp4) of how it looks in the simulator.

## Directory Structure

* **video:** Some screenshots
* **src:** Directory containing c++ source files along with Eigen library
* **CMakeLists.txt:** File containing compilation instructions
* **README.md:** Project readme file
* **install-mac.sh:** Script for installing uWebSockets on Macintosh
* **install-ubuntu.sh:** Script for installing uWebSockets on Ubuntu

## Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

`mkdir build && cd build`

`cmake .. && make` 

`./UnscentedKF`

Note that the programs that need to be written to accomplish the project are `src/ukf.cpp`, `ukf.h`, and `main.cpp` which will use some strategy to catch the car, just going to the cars current esimtated position will not be enough since the capture vehicle is not fast enough. There are a number of different strategies you can use to try to catch the car, but all will likely involve predicting where the car will be in the future which the UKF can do. Also remember that the run away car is simplifying moving a circular path without any noise in its movements.


Here is the main protocol that `main.cpp` uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the C++ program

// current noiseless position state of the capture vehicle, called hunter

["hunter_x"]

["hunter_y"]

["hunter_heading"]

// get noisy lidar and radar measurements from the run away car.

["lidar_measurement"]

["radar_measurement"]


**OUTPUT**: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["turn"] <= the desired angle of the capture car "hunter" no limit for the angle

["dist"] <= the desired distance to move the capture car "hunter" can't move faster than run away car



## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4
* uWebSocketIO

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`

## License

The content of this project is licensed under the [Creative Commons Attribution 3.0 license](https://creativecommons.org/licenses/by/3.0/us/deed.en_US).