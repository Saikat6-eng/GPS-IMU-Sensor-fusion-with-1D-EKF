# GPS-IMU-Sensor-fusion-with-1D-EKF

#gps-imu sensor fusion using 1D ekf..
Simple ekf based on it's equation and optimized for embedded systems.
#Tested on arm Cortex M7 microcontroller, archive 500hz running rate.

Input data to be needed -

Gps latitude, longitude - degrees(ned), alt - meter
Gps velocity - meters/s(ned)

#IMU Sensor frame
Acceleration_xyz - mss,
Gyro_xyz - radians,
Mag_xyz - uT,
 
https://youtu.be/GMFnxAAVidk

References -
https://youtu.be/6M6wSLD-8M8

https://github.com/slobdell/kalman-filter-example
