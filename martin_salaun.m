% ********************* Martin & salaun's algorithm ********************* %
%% load sensor data
close all;
clearvars;
load IMU_sensors_data.mat;
N = length(Time);
%% initial rotation
Gamma_ref = [0 0 1];
Mag_ref = [cos(64*pi/180) 0 sin(64*pi/180)];
lambda = 0.8;
x0 = [0,0,0];
options = optimoptions('fmincon','Display','none');
fun = @(x) lambda*norm(Gamma_ref'-(1/norm(Gamma(1,:)))*eul2rotm(x,'XYZ')*Gamma(1,:)')^2 + (1-lambda)*norm(Mag_ref'-(1/norm(Mag(1,:)))*eul2rotm(x,'XYZ')*Mag(1,:)')^2;
eul_angles0 = fmincon(fun,x0,[],[],[],[],[-pi,-pi/2,-pi],[pi,pi/2,pi],[],options);
%% normalize magnetic field
yb = Mag;
for i=1:N
    yb(i,:) = 1/norm(yb(i,:))*yb(i,:);
end
%% gain values
e = quaternion(1,0,0,0);
k = 1;
la = 0.01*(9*e-2);
ma = 0.01*(9*e-3);
lc = 0.01*(5*e-2);
mc = 0.01*(5*e-3);
%% inertial directions
A = quaternion(0,0,0,9.8);
C = quaternion(0,0,1,0);
%% compute measurements yc and convert to quaternions
ya = Gamma;
yc = zeros(N,3);
for i=1:N
    yc(i,:) = cross(ya(i,:),yb(i,:));
end
qya = quaternion([zeros(N,1),ya]);
qyc = quaternion([zeros(N,1),yc]);
%% initial quaternion and bias
qold = quaternion(eul_angles0,'euler','XYZ','frame');
bold = quaternion(0,0,0,0);
qnew = qold;
bnew = bold;
qw = quaternion([zeros(N,1),Omega]);
q_list = quaternion(zeros(N,4));
q_list(1) = qold;
%% simulation loop
for i=1:N-1
    % compute Ea and Ec
    Ea = A - qold*qya(i)*quatinv(qold);
    Ec = C - qold*qyc(i)*quatinv(qold);
    % compute gains matrices
    LaEa = 0.5*la*(A*Ea-Ea*A);
    MaEa = -0.5*ma*(A*Ea-Ea*A);
    CEc = 0.5*(C*Ec-Ec*C);
    LcEc = -0.5/(9.8^2)*lc*(CEc*A+A*CEc)*A;
    McEc = -0.5/(9.8^2)*mc*(CEc*A+A*CEc)*A;
    % update qi
    dt = Time(i+1)-Time(i);
    qnew = qold + dt*(0.5*qold*(qw(i)-bold)+(LaEa+LcEc)*qold+k*(1-norm(qold)^2)*qold);
    % update bias
    bnew = bold + dt*(quatinv(qold)*(MaEa+McEc)*qold);
    % save quaternion
    qold = qnew;
    bold = bnew;
    q_list(i+1) = qnew;
end
%% plot results
plot(Time,euler(q_list,'XYZ','frame'));
title('Attitude estimates');
legend('roll angle','pitch angle','yaw angle');