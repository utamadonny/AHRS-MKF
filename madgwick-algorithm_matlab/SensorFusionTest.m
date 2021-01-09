%% Tes Kodingan Sensor Fusion dengan HelperOrientationViewer.m
% openExample('shared_positioning/IMUOrientationExample')
% https://www.mathworks.com/help/fusion/ug/estimate-orientation-through-inertial-sensor-fusion.html#d120e3338

%% Akuisisi Data PASS

sensorData = load('IMUyaw2Donny.mat');
Fs=200;
save('SFTdiamBR1.mat');
clear;
clc;

%% Deklarasi  PASS
addpath('quaternion_library');clear all;close all;clc;
ld = load ('SFTdiamBR1.mat');
decim=2;
acc = ld.sensorData.Accelerometer; %ld.sensorData.Accelerometer(:,1) = ax
gyro = ld.sensorData.Gyroscope;
mag = ld.sensorData.Magnetometer;
time = (0:decim:size(acc,1)-1)/ld.Fs;  %hmmm ???
viewer = HelperOrientationViewer;
%% IMU Calibration
N = 1000;rng(1);
% accm = zeros(N,3); magm = zeros(N,3);
q = randrot(N,1); % uniformly distributed random rotations
imu = imuSensor('accel-mag');
% imum = imuSensor('accel-mag');
[~,x] = imu(acc(1:1000,:),mag(1:1000,:),q);
% [~,y] = imu(accm,magm,q);

scatter3(x(:,1),x(:,2),x(:,3));axis equal;title('Ideal Magnetometer Data');

%% IMU Cal2
c = [-50; 20; 100]; % ellipsoid center
r = [30; 20; 50]; % semiaxis radii

[x,y,z] = ellipsoid(c(1),c(2),c(3),r(1),r(2),r(3),20);
D = [mag(:,1),mag(:,2),mag(:,3)];
[A,b,expmfs] = magcal(D); % calibration coefficients
expmfs % Dipaly expected  magnetic field strength in uT
magC = (D-b)*A; % calibrated data
figure(1)
plot3(mag(:,1),mag(:,2),mag(:,3),'LineStyle','none','Marker','X','MarkerSize',8)
hold on
grid(gca,'on')
plot3(magC(:,1),magC(:,2),magC(:,3),'LineStyle','none','Marker', ...
            'o','MarkerSize',8,'MarkerFaceColor','r') 
axis equal
xlabel('uT')
ylabel('uT')
zlabel('uT')
legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off
% plot3(x(:,1),x(:,2),x(:,3))
% hold on
% grid(gca,'on')
% plot3(y(:,1),y(:,2),y(:,3)) 
% axis equal
% xlabel('uT')
% ylabel('uT')
% zlabel('uT')
% legend('Uncalibrated Samples', 'Calibrated Samples','Location', 'southoutside')
% title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
% hold off


% c = [-50; 20; 100]; % ellipsoid center
% r = [30; 20; 50]; % semiaxis radii
% 
% [x,y,z] = ellipsoid(c(1),c(2),c(3),r(1),r(2),r(3),20);
% D = [x(:),y(:),z(:)];
%% AHRS NED PASS

ifilt = ahrsfilter('SampleRate', ld.Fs);
for ii=1:size(acc,1)
    qahrs = ifilt(acc(ii,:), gyro(ii,:), mag(ii,:));
    viewer(qahrs);
    pause(0.01);
end

%% Compass PASS

ifilt = imufilter('SampleRate', 1000);
for ii=1:size(acc,1)
    qimu = ifilt(acc(ii,:), gyro(ii,:));
    viewer(qimu);
    pause(0.01);
end

