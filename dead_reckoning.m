% ********************** dead reckoning algorithm *********************** %

%% load sensor data
close all;
clearvars;
load IMU_sensors_data.mat;

%% initial rotation matrix from accelerometer and magnetometer
Gamma_ref = [0 0 1];
Mag_ref = [cos(64*pi/180) 0 sin(64*pi/180)];
lambda = 0.8;
fun = @(x) lambda*norm(Gamma_ref'-(1/norm(Gamma(1,:)))*eul2rotm(x,'XYZ')*Gamma(1,:)')^2 ...
    + (1-lambda)*norm(Mag_ref'-(1/norm(Mag(1,:)))*eul2rotm(x,'XYZ')*Mag(1,:)')^2;
x0 = [0,0,0];
eul_angles0 = fminsearch(fun,x0);
R_0 = eul2rotm(eul_angles0,'XYZ');

%% integration
N = length(Time);
eul_angles = zeros(N,3);
eul_angles(1,:) = eul_angles0;
R_old = R_0;
R_new = R_0;
for k=1:N-1
    dt = Time(k+1) - Time(k);
    angular_rate_matrix = [0 -Omega(k,3) Omega(k,2);
                           Omega(k,3) 0 -Omega(k,1);
                           -Omega(k,2) Omega(k,1) 0];
    R_new = R_old*(eye(3)+dt*angular_rate_matrix);
    eul_angles(k+1,:) = rotm2eul(R_new,'XYZ');
    R_old = R_new;
end

%% figures plot
figure(1);
plot(Time,eul_angles);
title('Attitude estimates');
legend('roll angle', 'pitch angle', 'yaw angle');