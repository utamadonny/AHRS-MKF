% ExampleScript.m
%
% This script demonstrates use of the MadgwickAHRS and MahonyAHRS algorithm
% classes with example data. ExampleData.mat contains calibrated gyroscope,
% accelerometer and magnetometer data logged from an AHRS device (x-IMU)
% while it was sequentially rotated from 0 degrees, to +90 degree and then
% to -90 degrees around the X, Y and Z axis.  The script first plots the
% example sensor data, then processes the data through the algorithm and
% plots the output as Euler angles.
%
% Note that the Euler angle plot shows erratic behaviour in phi and psi
% when theta approaches ±90 degrees. This due to a singularity in the Euler
% angle sequence known as 'Gimbal lock'.  This issue does not exist for a
% quaternion or rotation matrix representation.
%
% Date          Author          Notes
% 28/09/2011    SOH Madgwick    Initial release
% 13/04/2012    SOH Madgwick    deg2rad function no longer used
% 06/11/2012    Seb Madgwick    radian to degrees calculation corrected

%% Start of script

addpath('quaternion_library');      % include quaternion library
% addpath('Data');                  % add data
%% Clear 
% close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal

%% (1) Import and plot sensor data

% load('Data/5v.mat');% load('Data/2021-08-20 16-51-34.mat'); %Beta = 5
% load('Data/2v.mat');% load('Data/2021-09-08 15-42-01'); %Beta = 2
% load('Data/1v.mat');% load('Data/2021-08-20 16-33-38.mat'); %Beta = 1
% load('Data/0c5v.mat');   % 1.0921 |0.0570 |0.0852 | 52
% load('Data/0c5v0.mat');  % 0.3862 |0.1387 |0.0483 | 53
% load('Data/0c5v1.mat');  % 0.5399 |0.1218 |0.0762 | 49
% load('Data/0c5v2.mat');  % 0.9983 |0.1726 |0.0542 | 65
% load('Data/0c5v3.mat');  % 0.7166 |0.0854 |0.0533 | 10
% load('Data/0c5v4.mat');  % 0.2174 |0.1159 |0.0382 | 72
% load('Data/0c5v5.mat');  % 0.5318 |0.1283 |0.0297 | 95 
% load('0c1v.mat');% load('Data/2021-08-17 11-14-48.mat'); %Beta = 0.1
% load('Data/0c1v0.mat');
% load('Data/2021-09-08 17-19-10'); %Beta = 0.05 ga dapet
% load('Data/2021-09-08 15-58-12'); %Beta = 0.01 ga dapet kurang data
% load('Data/2021-07-08 22-03-03.mat'); % salah, Hz =100
% load('Data/2021-07-16 21-43-46.mat'); % yang katanya ada delay, Hz=100 Beta=0.1
% Note: beta arduino = 5 ==> beta matlab =0.5
% Gyroscope = ld.sensorData.AngularVelocity;
% Accelerometer = ld.sensorData.Acceleration;
% Magnetometer=ld.sensorData.MagneticField;
% time= 1:1:1600;
% Fs=200;
% time = (0:decim:size(Accelerometer,1)-1)/Fs;
%% (2) Preprocess to NED
%  filter.update(gyr.y(),gyr.x(),-gyr.z(),-acc.y(),-acc.x(),acc.z(),-hag.y(),-hag.x(),hag.z());
%  working
Gyroscope1=Gyroscope;
Accelerometer1=Accelerometer;
Magnetometer1=Magnetometer;
% Gyroscope1(:,1)=-1*Gyroscope(:,1);
% Gyroscope1(:,2)=-1*Gyroscope(:,2);
Gyroscope1(:,3)=-1*Gyroscope(:,3);
Accelerometer1(:,1)=-1*Accelerometer(:,1);
Accelerometer1(:,2)=-1*Accelerometer(:,2);
% Accelerometer1(:,3)=-1*Accelerometer(:,3);
Magnetometer1(:,1)=-1*Magnetometer(:,1);
Magnetometer1(:,2)=-1*Magnetometer(:,2);
% Magnetometer1(:,3)=-1*Magnetometer(:,3);
%% (2a) tuker axis
Magnetometer1 = tuker(Magnetometer1,1,2);
Accelerometer1=tuker(Accelerometer1,1,2);
Gyroscope1=tuker(Gyroscope1,1,2);
%% (3) Plot Sensor Data
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% (4) Process sensor data through algorithm
SamplePeriode = 100;
BetaQ= 0.05;
% Kp= 10; Ki=0;
% AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
AHRS = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', BetaQ); % Tuning Sampleperiod and Beta
% AHRS = MahonyAHRS('SamplePeriod', 1/SamplePeriode, 'Kp', Kp, 'Ki',Ki);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
%     AHRS.Update(Gyroscope1(t,:) * (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));
%     AHRS.UpdateIMU(Gyroscope1(t,:) * (pi/180), Accelerometer1(t,:)); %, Magnetometer1(t,:));
    AHRS.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% (5) Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem c ommonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
% euler = quatern2euler(quaternConj(quaternion));
offset = 180;
euler(:,3)=euler(:,3)+offset; % yaw membutuhkan offset
euler=tuker(euler,1,3); % menukar roll dan yaw, agar formatnya sama seperti data real dan realtime
euler(:,2:3)=-1*euler(:,2:3); % Switch for NED 
euler=tuker(euler,2,3); % NED roll dan pitch tertukar
%% (6) Plot Euler
figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,3), 'r'); %roll
plot(time, euler(:,2), 'g'); %pitch
plot(time, euler(:,1), 'b'); %yaw
plot(time, aeuler(:,3)); %roll
plot(time, aeuler(:,2)); %pitch
plot(time, aeuler(:,1)); %yaw
plot(time, reuler(:,3)); %roll
plot(time, reuler(:,2)); %pitch
plot(time, reuler(:,1)); %yaw
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
% legend('\phi', '\theta', '\psi')
% legend('\phi', '\theta', '\psi','a\phi', 'a\theta', 'a\psi');
legend('matlab\phi', 'matlab\theta', 'matlab\psi','arduino\phi', 'arduino\theta', 'arduino\psi','real\phi', 'real\theta', 'real\psi');
hold off;

%% End of script
% sta = 1911; stb = length(time);
rmsey = sqrt(mean((euler(sta:stb,1) - reuler(sta:stb,1)).^2));
rmsep = sqrt(mean((euler(sta:stb,2) - reuler(sta:stb,2)).^2));
rmser = sqrt(mean((euler(sta:stb,3) - reuler(sta:stb,3)).^2));
amsey = sqrt(mean((euler(sta:stb,1) - aeuler(sta:stb,1)).^2));
amsep = sqrt(mean((euler(sta:stb,2) - aeuler(sta:stb,2)).^2));
amser = sqrt(mean((euler(sta:stb,3) - aeuler(sta:stb,3)).^2));

% rmsey = sqrt(mean((euler(:,1) - reuler(:,1)).^2));
% rmsep = sqrt(mean((euler(:,2) - reuler(:,2)).^2));
% rmser = sqrt(mean((euler(:,3) - reuler(:,3)).^2));

rmsey
rmsep
rmser
amsey
amsep
amser

%% Save to CSV
save('dummy.mat');
fd= load('dummy.mat');
T1=array2table(fd.euler);
T1.Properties.VariableNames(1:3) = {'myaw','mpitch','mroll'};
T2=array2table(fd.reuler);
T2.Properties.VariableNames(1:3) = {'yaw','pitch','roll'};
T3=array2table(fd.aeuler);
T3.Properties.VariableNames(1:3) = {'ayaw','apitch','aroll'};
timet=array2table(time);
% writematrix( 
Nums=[T2 T3 T1 timet];
writetable(Nums,'2021-07-02 08-47-47M1.csv');
% Nums=[reuler meuler euler time];
% writematrix(Nums,'dummy.csv')
% csvwrite('2021-07-16 21-43-46M.csv',filedata.reuler,filedata.meuler,filedata.euler)