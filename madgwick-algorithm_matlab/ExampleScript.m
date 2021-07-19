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
% addpath('Data');                    % add data
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data

load('Data/2021-07-16 21-43-46.mat');
% Gyroscope = ld.sensorData.AngularVelocity;
% Accelerometer = ld.sensorData.Acceleration;
% Magnetometer=ld.sensorData.MagneticField;
% time= 1:1:1600;
% Fs=200;
% time = (0:decim:size(Accelerometer,1)-1)/Fs;
%% Preprocess to NED
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
%% tuker axis
Magnetometer1 = tuker(Magnetometer1,1,2);
Accelerometer1=tuker(Accelerometer1,1,2);
Gyroscope1=tuker(Gyroscope1,1,2);
%% Plot
% figure('Name', 'Sensor Data');
% axis(1) = subplot(3,1,1);
% hold on;
% plot(time, Gyroscope(:,1), 'r');
% plot(time, Gyroscope(:,2), 'g');
% plot(time, Gyroscope(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Angular rate (deg/s)');
% title('Gyroscope');
% hold off;
% axis(2) = subplot(3,1,2);
% hold on;
% plot(time, Accelerometer(:,1), 'r');
% plot(time, Accelerometer(:,2), 'g');
% plot(time, Accelerometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Acceleration (g)');
% title('Accelerometer');
% hold off;
% axis(3) = subplot(3,1,3);
% hold on;
% plot(time, Magnetometer(:,1), 'r');
% plot(time, Magnetometer(:,2), 'g');
% plot(time, Magnetometer(:,3), 'b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Flux (G)');
% title('Magnetometer');
% hold off;
% linkaxes(axis, 'x');

%% Process sensor data through algorithm
SamplePeriode = 512;
BetaQ= 0.1;
% AHRS = MadgwickAHRS('SamplePeriod', 1/256, 'Beta', 0.1);
AHRS = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', BetaQ); % Tuning Sampleperiod and Beta
% AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope1(t,:) * (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem c ommonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
offset = 180;
euler(:,3)=euler(:,3)+offset; % yaw membutuhkan offset
euler=tuker(euler,1,3); % menukar roll dan yaw, agar formatnya sama seperti data real dan realtime
euler(:,2:3)=-1*euler(:,2:3); % Switch for NED 
euler=tuker(euler,2,3); % NED roll dan pitch tertukar
%% Plot Euler
figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,3), 'r'); %roll
plot(time, euler(:,2), 'g'); %pitch
plot(time, euler(:,1), 'b'); %yaw
plot(time, meuler(:,3)); %roll
plot(time, meuler(:,2)); %pitch
plot(time, meuler(:,1)); %yaw
plot(time, reuler(:,3)); %roll
plot(time, reuler(:,2)); %pitch
plot(time, reuler(:,1)); %yaw
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
% legend('\phi', '\theta', '\psi','m\phi', 'm\theta', 'm\psi');
legend('matlab\phi', 'matlab\theta', 'matlab\psi','arduino\phi', 'arduino\theta', 'arduino\psi','real\phi', 'real\theta', 'real\psi');
hold off;

%% End of script
rmsey = sqrt(mean((euler(1:3000,1) - reuler(1:3000,1)).^2));
rmsep = sqrt(mean((euler(1:3000,2) - reuler(1:3000,2)).^2));
rmser = sqrt(mean((euler(1:3000,3) - reuler(1:3000,3)).^2));
rmsey
rmsep
rmser

%% Save to CSV
save('dummy.mat');
fd= load('dummy.mat');
T1=array2table(fd.euler);
T1.Properties.VariableNames(1:3) = {'myaw','mpitch','mroll'};
T2=array2table(fd.reuler);
T2.Properties.VariableNames(1:3) = {'yaw','pitch','roll'};
T3=array2table(fd.meuler);
T3.Properties.VariableNames(1:3) = {'ayaw','apitch','aroll'};
timet=array2table(time);
% writematrix( 
Nums=[T2 T3 T1 timet];
writetable(Nums,'2021-07-16 21-43-46M1.csv');
% Nums=[reuler meuler euler time];
% writematrix(Nums,'dummy.csv')
% csvwrite('2021-07-16 21-43-46M.csv',filedata.reuler,filedata.meuler,filedata.euler)