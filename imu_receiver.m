function [imuFs] = imu_receiver(imuFs)
    imu = imuSensor('accel-gyro', ...
    'ReferenceFrame', 'ENU', 'SampleRate', imuFs);
    
    % Accelerometer
    imu.Accelerometer.MeasurementRange =  19.6133;
    imu.Accelerometer.Resolution = 0.0023928;
    imu.Accelerometer.NoiseDensity = 0.0012356;
    
    % Gyroscope
    imu.Gyroscope.MeasurementRange = deg2rad(250);
    imu.Gyroscope.Resolution = deg2rad(0.0625);
    imu.Gyroscope.NoiseDensity = deg2rad(0.025);
