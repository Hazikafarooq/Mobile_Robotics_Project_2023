Pose Estimation of a Mobile Robot Based on Fusion of IMU Data and GPS data Using an Extended Kalman Filter

We are estimating the position and orientation of ground vehicles by fusing data from an inertial measurement unit (IMU) and a global positioning system (GPS) receiver using the follownig procedure,
![image](https://github.com/Hazikafarooq/Mobile_Robotics_Project_2023/assets/84680497/5faf82df-6cbb-467f-8528-704e7415fccc)

|
|---> IMU (Inertial Measurement Unit)
|        |
|        |---> Accelerometer
|        |        |---> Measures linear acceleration
|        |        |---> Provides data for estimating velocity and position
|        |
|        |---> Gyroscope
|        |        |---> Measures angular velocity
|        |        |---> Provides data for estimating orientation
|
|---> GPS (Global Positioning System) Receiver
|        |---> Provides global position coordinates (latitude, longitude, altitude)
|        |---> Offers low-frequency, high-precision positional data
|
|---> Sensor Fusion Module
|        |
|        |---> Extended Kalman Filter (EKF)
|        |        |---> Fuses IMU and GPS data
|        |        |---> Estimates orientation and position
|        |        |---> Corrects for sensor noise and biases
|        |
|        |---> Process Model
|        |        |---> Defines how the state of the system changes over time
|        |        |---> Used in EKF prediction step
|        |
|        |---> Measurement Model
|                 |---> Maps the true state space into the observed space
|                 |---> Used in EKF update step
|
|---> Estimated Pose Output
         |---> Orientation (quaternion)
         |---> Position (x, y, z coordinates)
