Pose Estimation of a Mobile Robot Based on Fusion of IMU Data and GPS data Using an Extended Kalman Filter

We are estimating the position and orientation of ground vehicles by fusing data from an inertial measurement unit (IMU) and a global positioning system (GPS) receiver using the follownig procedure,

![image](https://github.com/Hazikafarooq/Mobile_Robotics_Project_2023/assets/84680497/5faf82df-6cbb-467f-8528-704e7415fccc)

Simulation Setup

Set the sampling rates. In a typical system, the accelerometer and gyroscope in the IMU run at relatively high sample rates. The complexity of processing data from those sensors in the fusion algorithm is relatively low. Conversely, the GPS runs at a relatively low sample rate and the complexity associated with processing it is high. In this fusion algorithm the GPS samples are processed at a low rate, and the accelerometer and gyroscope samples are processed together at the same high rate.
To simulate this configuration, the IMU (accelerometer and gyroscope) is sampled at 100 Hz, and the GPS is sampled at 10 Hz.

Fusion Filter

Create the filter to fuse IMU + GPS measurements. The fusion filter uses an extended Kalman filter to track orientation (as a quaternion), position, velocity, and sensor biases.
The insfilterNonholonomic object has two main methods: predict and fusegps. The predict method takes the accelerometer and gyroscope samples from the IMU as input. Call the predict method each time the accelerometer and gyroscope are sampled. This method predicts the states forward one time step based on the accelerometer and gyroscope. The error covariance of the extended Kalman filter is updated in this step.
The fusegps method takes the GPS samples as input. This method updates the filter states based on the GPS sample by computing a Kalman gain that weights the various sensor inputs according to their uncertainty. An error covariance is also updated in this step, this time using the Kalman gain as well.
The insfilterNonholonomic object has two main properties: IMUSampleRate and DecimationFactor. The ground vehicle has two velocity constraints that assume it does not bounce off the ground or slide on the ground. These constraints are applied using the extended Kalman filter update equations. These updates are applied to the filter states at a rate of IMUSampleRate/DecimationFactor Hz.

Create Ground Vehicle Trajectory

The waypointTrajectory object calculates pose based on specified sampling rate, waypoints, times of arrival, and orientation. Specify the parameters of a circular trajectory for the ground vehicle

GPS Receiver

Set up the GPS at the specified sample rate and reference location. The other parameters control the nature of the noise in the output signal.

IMU Sensors

Typically, ground vehicles use a 6-axis IMU sensor for pose estimation. To model an IMU sensor, define an IMU sensor model containing an accelerometer and gyroscope. In a real-world application, the two sensors could come from a single integrated circuit or separate ones. The property values set here are typical for low-cost MEMS sensors.

Initialize the Variances of the insfilterNonholonomic

The measurement noises describe how much noise is corrupting the GPS reading based on the gpsSensor parameters and how much uncertainty is in the vehicle dynamic model.
The process noises describe  how well the filter equations describe the state evolution. Process noises are determined empirically using parameter sweeping to jointly optimize position and orientation estimates from the filter. 

Initialize Scopes

The HelperScrollingPlotter scope enables plotting of variables over time. It is used here to track errors in pose. The HelperPoseViewer scope allows 3-D visualization of the filter estimate and ground truth pose. The scopes can slow the simulation. To disable a scope, set the corresponding logical variable to false.

Simulation Loop
The main simulation loop is a while loop with a nested for loop. The while loop executes at the gpsFs, which is the GPS measurement rate. The nested for loop executes at the imuFs, which is the IMU sample rate. The scopes are updated at the IMU sample rate.

Error Metric Computation
Position and orientation were logged throughout the simulation. Now compute an end-to-end root mean squared error for both position and orientation
