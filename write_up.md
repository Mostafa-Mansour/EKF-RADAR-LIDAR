# Code organization
---
<<<<<<< HEAD
[image1]: ./images/data.png "data"
=======
[image1]: ./data.png "data"
>>>>>>> ab626eedb563564f9ae163e358001a71ab3dce51
The data file can be checked [here](obj_pose-laser-radar-synthetic-input.txt).

* ![alt text][image1]
* Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

* For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

* For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

* Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

* I will use the measurement values and timestamp in Kalman filter algorithm. Groundtruth, which represents the actual path the bicycle took, is for calculating root mean squared error.
<<<<<<< HEAD

* Please, make sure to write the full path of the data file in your machine. It is important for the code to be compiled and run.
=======
>>>>>>> ab626eedb563564f9ae163e358001a71ab3dce51
---
The project consists of Three main classes:
* Measurement class
* ExtendedKalmanFilter class
* Tracking class
---
Measurement class
* contains three private members, timestamp, enumeration of sensor type and raw measurements.
---
ExtendedKalmanFilter class
* contains the parameters and the functions of Kalmn filter.
--- 
Tracking class
<<<<<<< HEAD
* In which fusing the measurements occurs. 
=======
* In which fusing the measurements occurs. 
>>>>>>> ab626eedb563564f9ae163e358001a71ab3dce51
