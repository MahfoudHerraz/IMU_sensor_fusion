% ********************* Hamel and Mahony's algorithm ******************** %

%% load sensor data
close all;
clearvars;
load IMU_sensors_data.mat;

%% calculate initial estimates
Gamma_ref = [0 0 1];
Mag_ref = [cos(64*pi/180) 0 sin(64*pi/180)];
lambda = 0.8;
options = optimoptions('fmincon','Display','none');
fun = @(x) lambda*norm(Gamma_ref'-(1/norm(Gamma(1,:)))*eul2rotm(x,'XYZ')*Gamma(1,:)')^2 + (1-lambda)*norm(Mag_ref'-(1/norm(Mag(1,:)))*eul2rotm(x,'XYZ')*Mag(1,:)')^2;
x0 = [0,0,0];
eul_angles0 = fmincon(fun,x0,[],[],[],[],[-pi,-pi/2,-pi],[pi,pi/2,pi],[],options);
R_0 = eul2rotm(eul_angles0,'XYZ');
b_0 = [0 0 0];

%% gain values
k1 = 0.8;
k2 = 0.2;
kp = 10; 
kI = 3;
%% simulation loop
N = length(Time);
eul_angles = zeros(N,3);
eul_angles(1,:) = eul_angles0;
R_old = R_0;
R_new = R_0;
b_old = b_0;
b_new = b_0;
wk_list = zeros(N,3);
Emes_list = zeros(N-1,1);
for k = 1:N-1
    % estimated measures
    Gamma_est = transpose(R_old)*Gamma_ref';
    Mag_est = transpose(R_old)*Mag_ref';
    % calculate error
    Emes_list(k) = k1*(1-(1/norm(Gamma(k,:)))*Gamma(k,:)*Gamma_est)+ ...
        k2*(1-(1/norm(Mag(k,:)))*Mag(k,:)*Mag_est);
    % update wk
    wk = -kp*(k1*cross(Gamma_est,(1/norm(Gamma(k,:)))*Gamma(k,:)) ...
        + k2*cross(Mag_est,(1/norm(Mag(k,:)))*Mag(k,:)));
    wk_list(k+1,:) = wk;
    % update bk
    dt = Time(k+1) - Time(k);
    b_new = b_old - dt*kI/kp*wk;
    % update Rk using Rodrigues formula
    angular_rate_vect = Omega(k,:)-b_new+wk;
    angular_rate_matrix = [0 -angular_rate_vect(3) angular_rate_vect(2);
                           angular_rate_vect(3) 0 -angular_rate_vect(1);
                           -angular_rate_vect(2) angular_rate_vect(1) 0];
    vnorm = norm(angular_rate_vect);
    Ak = eye(3) + sin(vnorm*dt)/vnorm*angular_rate_matrix ...
        + (1-cos(vnorm*dt))/(vnorm^2)*angular_rate_matrix*angular_rate_matrix;
    R_new = R_old*Ak;
    % euler angles from matrix
    eul_angles(k+1,:) = rotm2eul(R_new,'XYZ');
    % loop update
    R_old = R_new;
    b_old = b_new;
end

%% figures plot
figure(1);
plot(Time,eul_angles);
title('Attitude estimates');
legend('roll angle', 'pitch angle', 'yaw angle');
% figure(2);
% plot(Time,wk_list);
% title('wk');
% legend('x axis', 'y axis', 'z axis');
figure(3);
plot(Time(1:N-1),Emes_list);
title('error evolution');