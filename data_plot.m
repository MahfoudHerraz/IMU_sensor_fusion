% ******************************* data plots **************************** %
%% load sensor data
close all;
clearvars;
load IMU_sensors_data.mat;

%% acceleration plots
figure(1);
subplot(3,1,1);
plot(Time,Gamma(:,1));
title('Gamma_x');
subplot(3,1,2);
plot(Time,Gamma(:,2));
title('Gamma_y');
subplot(3,1,3);
plot(Time,Gamma(:,3));
title('Gamma_z');
saveas(figure(1),'gamma.png');
%% magnetic field plots
figure(2);
subplot(3,1,1);
plot(Time,Mag(:,1));
title('Mag_x');
subplot(3,1,2);
plot(Time,Mag(:,2));
title('Mag_y');
subplot(3,1,3);
plot(Time,Mag(:,3));
title('Mag_z');
saveas(figure(2),'mag.png');
%% angular rates plots
figure(3);
subplot(3,1,1);
plot(Time,Omega(:,1));
title('roll rate');
subplot(3,1,2);  
plot(Time,Omega(:,2));
title('pitch rate');
subplot(3,1,3);
plot(Time,Omega(:,3));
title('yaw rate');
saveas(figure(3),'omega.png');