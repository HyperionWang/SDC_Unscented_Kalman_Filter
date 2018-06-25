# Unscented Kalman Filter Project Document
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./Doc/UnscentedFilterFlow.png
[image2]: ./Doc/AugumentedSigmaMatrix.png
[image3]: ./Doc/AugmentedMatrix.png
[image4]: ./Doc/SigmaPredictions.png
[image5]: ./Doc/WeightedAverage.png
[image6]: ./Doc/MeasurementPredictions.png
[image7]: ./Doc/MeasurementUpdate.png
[image8]: ./Doc/RMSE_Results_Data1.png
[image9]: ./Doc/RMSE_Results_Data2.png
[image10]: ./Doc/NIS_table.png
[image11]: ./Doc/NISLaser1.png
[image12]: ./Doc/NISLaser2.png
[image13]: ./Doc/NISRadar1.png
[image14]: ./Doc/NISRadar2.png

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The finished C++ code could compile without errors, and the testing results showed that position and velocity RMSE [0.0788, 0.0935, 0.3504, 0.2717] from the algorithm outputs are less than [0.09, 0.1, 0.40, 0.30]. 

The implemented Unscented Kalman Filter is able to initialize, predict, and update the measurement based on the noisy Laser and Radar measurement results. 

The simulator version used is V1.45: Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases) The code is tested on Windows System using Windows 10 Bash on Ubuntu by installed uWebSocketIO.


The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

The implemented Unscented Kalman Filter, RMSE calculation, and NIS log file output are in src/ukf.cpp, src/ukf.h, tools.cpp, tools.h, and src/main.cpp. 


Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]


### Unscented Kalman Filter Description

#### Filter Work Flow

##### 1. Filter Initialization

As the class UKF() is initialized, the option to use laser and radar data is set, State Vector x = [px, py, v, yaw, yaw_d] and corresponding covariance matrix is created (waiting for the first measurement data for initialization), and the process noise/measurement noise are set. 

In the function UKF::Processmeasurement (measurement), the filter's process flow is implemented. First, if the filter is not initialized yet, the filter would use the first input measurement data to update the px, py. If the measurement is from Laser, the measurement data could be used directly; if the measurement is from Radar, the measurement data need convert from Polar Coordinate to Cartesian Coordinate, and then update px and py. Also, the timestamp for the filter is updated.

The value choice of parameters on process noise on acceleration, process noise on yaw acceleration, initial value of x_ state, and covariance would affect the RMSE results. The basic idea is to choose the proper value on std_a and std_yaw so that prediction could handle the variation on the velocity and yaw acceleration, but not to make the prediction jump too much. I used 3 for std_a_, and 1 for std_yawdd_. The intial value on the covariance P_ also need to tune in order to make the intial value of RMSE not too large. For covariance on velocity and yaw, I used pretty large values 130 for velocity, and 100 for yaw since the initial value would be varying a lot from test to test. By using that, the RMSE are pretty good on both test data 1 and test data 2.

##### 2. Prediction and Measurement Update
As more measurement data come in to the filter, the Unscent Kalman Filter starts the repetitive flow in (1) Predict and (2) Measurement Update, which will be explained in the following sections. 

In the prediction part, both Radar and Laser Measuremnt would use the same prediction algorithm, since the only input for the prediction is the time stamp different delta_t. 

In the measurement update part, the update flow is different between Radar measurement and Laser Measurement. 

##### 3. Report results and calculate RMSE, NIS in Radar, and NIS in Laser
After the prediction and measurement, the estimated px, py, vx, and vy would be compared with the ground truth data in order to evaluate the performance of the designed filter. Also, the NIS showing the performance of selected process noise and initial value (std_a_, std_yawdd_, initial x_, and initial P_) are saved in the log file "build/laserNIS.csv" and "build/radarNIS.csv".


#### State Predictions Function Descriptions

The implementation of the state predition is in the function UKF::Prediction(delta_t) in src/ukf.cpp.

The predition in Unscented Kalman Filter consists three steps: (1) Generate Sigma Points (Augmented); (2) Predict Sigma Points results using CTRV motion model and corresponding formula; and (3) Predict Mean and Covariance based on the predicted Sigma Points. The implementation is from Line 154 to 254 in ukf.cpp.

![alt text][image1]

##### Generate Sigma Points using Augmented Sigma Matrix

The basic idea for generating Sigma points in Unscented Kalman Filter is to use representative sampling points to predict the next State and Covariance instead of using State Transition Matrix F in the Linear Kalman Filter. The way to create the sigma matrix is using variation paramters lamda and square root of covariance matrix to generate the representative sampling points around the previous state and close to the variation region. The formula is as following.
  
![alt text][image2]

The Augmentaed State is to consider the variation as the part of the state matrix as well. 

![alt text][image3]

##### Predict Sigma Points using CTRV motion model

Once the Augmented Sigma Points generated, the prediction of the next state of those sigma points could be calcualted based on the current state value [px, py, v, yaw, yaw_d, velocity acceleration, yaw acceleration] and the time different delta_t. The formula is based on the assumption using CTRV motion model. After the prediction, the sigma prediction points are in [px, py, v, yaw, yaw_d].

![alt text][image4]

##### Predict Next State using Mean and Variations

After the Prediction on Sigma Points, the prediction on the next state is straightforward. Use the weighted average of all sigma points as the prediction on next state, and use the weighted average of the varition sqaure (in matrix format) as the covariance of the next state. The yaw need the angle wrapping to make sure the value is within [-pi, pi] by adding or subtracting 2*pi. At the end of this step, the prediction on next state x_(k+1|k) and P_(K+1|k) is updated.

![alt text][image5]

#### Measurement Update Function Descriptions

The implementation of the measurement update is in the functions UKF::UpdateLidar(measurement) and UKF::UpdateRadar(measurement) in src/ukf.cpp.

The measurement update consists two parts: (1) predict the measurment; (2) Update the state based on the latest measurement. 

##### Measurement Prediction
The difference between Laser Data Measuremnet Update and Radar Data Measurmeent lies in the type of the measurement. In the laser measurement, the data is [px, py], which could directly compare with the state x [px, py, v, yaw, yaw_d]. For radar measurement, the data is in polar coordinate with [rho, theta, rho_d], so that the sigma predictions need to convert in to Polar coordinate in order to make the measurement predictions. 

The prediction is basically the same as the Sigma Prediction Matrix, but in the format matching with the measurement data. And then, the weighted average is the measurement prediction mean, and the variation from the mean is used to calcualate the covariance S(k+1|k). Please note that the measurement process variation (R) is also added into the measurement prediction covariance (S). 

![alt text][image6]

For Laser Measurement, the dimension for measurement prediction z(k+1|k) is 2 (px and py). For Radar Measurement, the dimension for measurement prediction z(k+1|k) is 3 (rho, theta, rho_d).

##### Measurement Update

The measurement update is done through the Kalman gain matrix K, which is built based on the cross-correlation matrxi T(k+1|k) between the measurement prediction variation and state prediction variation. 

![alt text][image7]

After the measurement update, the updated state and covariance matrix are used to calculate the RMSE and NIS, in order to evaluate the performance of designed Kalman Filter.

#### Results and Summary

The designed Unscented Kalman Filter compile correctly, and RMSE of px, py, vx and vy meet the requirement for both data 1 ([0.0788, 0.0935, 0.3504, 0.2717]) and data 2 ([0.0731, 0.0733, 0.4789, 0.2492]). The RMSE results are better then Extended Kalman Filter for all four RMSEs.

![alt text][image8]
![alt text][image9]

The following are NIS value over updates for Laser and Radar Measurement. The 0.050 value line for laser data is 5.991, and for radar is 7.815. As shown in the following plots, both laser and radar updats' NISs are mostly below the 0.050 value line.

![alt text][image10]

![alt text][image11]

![alt text][image12]

![alt text][image13]

![alt text][image14]

The parameters chose for the varition and intial state value are as following:

```c++

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;
  
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
 
  // Lamda for generating Augmented Sigma Matrix
  lambda_ = 3 - n_aug_;
  
  //Initial value for state
  x_ << 0, 0, 2, 0.3, 0;
  
  //Initial covariance for state
  P_ <<   1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 130, 0, 0,
          0, 0, 0, 100, 0,
          0, 0, 0, 0, 1;

```

In sum, a unscented Kalman filter is designed and implemented in oder to realize real-time object movement tracking and predictions with low error RMSE. The designed unscented Kalman Filter has function blocks to initialize filter, make prediction on position, velocity, moving direction, and direction change, and update the prediction state based on the updated measurement from Laser and Radar sensors' data. 

In the next step, I will continue with the Bonus Challenge to catch the Run Away car using Unscented Kalman Filter to predict and update the states of both cars, in order to make the catch successful. 
