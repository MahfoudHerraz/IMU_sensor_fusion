% **************** inversion from accelero and magneto ****************** %

%% load sensor data
close all;
clearvars;
load IMU_sensors_data.mat;

%% reference inertial directions
Gamma_ref = [0 0 1];
Mag_ref = [cos(64*pi/180) 0 sin(64*pi/180)];
lambda = 0.8;
N = length(Time);
eul_angles = zeros(N,3);
%% inversion loop
x0 = [0,0,0];
options = optimoptions('fmincon','Display','none');
fun = @(x) lambda*norm(Gamma_ref'-(1/norm(Gamma(1,:)))*eul2rotm(x,'XYZ')*Gamma(1,:)')^2 + (1-lambda)*norm(Mag_ref'-(1/norm(Mag(1,:)))*eul2rotm(x,'XYZ')*Mag(1,:)')^2;
eul_angles(1,:) = fmincon(fun,x0,[],[],[],[],[-pi,-pi/2,-pi],[pi,pi/2,pi],[],options);
for k=2:N
    fun_k = @(x) lambda*norm(Gamma_ref'-(1/norm(Gamma(k,:)))*eul2rotm(x,'XYZ')*Gamma(k,:)')^2 + (1-lambda)*norm(Mag_ref'-(1/norm(Mag(k,:)))*eul2rotm(x,'XYZ')*Mag(k,:)')^2;
    eul_angles(k,:) = fmincon(fun_k,eul_angles(k-1,:),[],[],[],[],[-pi,-pi/2,-pi],[pi,pi/2,pi],[],options);
end

%% figures plot
figure(1);
plot(Time,eul_angles);
title('Attitude estimates');
legend('roll angle', 'pitch angle', 'yaw angle');