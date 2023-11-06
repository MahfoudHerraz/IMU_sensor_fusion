% ************************* complementary filter ************************ %

%% load sensor data
close all;
clearvars;
load IMU_sensors_data.mat;

%% complementary filter
N = length(Time);
sample_rate = N/Time(N);
comp_filt = complementaryFilter("SampleRate",sample_rate);
q = comp_filt(Gamma,Omega,Mag);

%% plot results
plot(Time,euler( q, 'XYZ', 'frame'));
title('Attitude estimates');
legend('roll angle', 'pitch angle', 'yaw angle');