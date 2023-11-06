# IMU_sensor_fusion
This repository contains different algorithms for attitude estimation (roll, pitch and yaw angles) from IMU sensors data: accelerometer, magnetometer and gyrometer measurements

File 'IMU_sensors_data.mat' contains real-life sensors measurements, which can be plotted by running the file 'data_plot.m'

File 'accel_mag_inversion.m' solves an opptimization (inverse) problem to find atttitude angles that minimize the error between expected and measured acceleration and magnetic field

File 'dead_reckoning.m' integrates angular rates from gyrometer measurements, with a given initial condition, to obtain attitude angles

File 'complementary_filter.m' uses Matlab function 'complementaryFilter()' to estimate attitude angles from IMU measurements

File 'hamel_mahony.m' implements the nonlinear complementary filter proposed by 'Hamel, Tarek, and Robert Mahony. "Attitude estimation on SO(3) based on direct inertial measurements." Proceedings 2006 IEEE International Conference on Robotics and Automation, 2006. ICRA 2006.. IEEE, 2006.'.

File 'martin_salaun.m' implements the invariant observer proposed in 'Martin, Philippe, and Erwan Salaun. "Invariant observers for attitude and heading estimation from low-cost inertial and magnetic sensors." 2007 46th IEEE Conference on Decision and Control. IEEE, 2007'.
