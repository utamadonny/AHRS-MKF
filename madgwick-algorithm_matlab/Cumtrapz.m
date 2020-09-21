%% load data
clear all
close all
clc
%% Algoritma Integral Cumtrapz
% https://www.mathworks.com/help/matlab/ref/cumtrapz.html

load('new.mat');
%% Design High Pass Filter
fs = 8000; % Sampling Rate
fc = 0.1/30;  % Cut off Frequency
order = 6; % 6th Order Filter
%% accelerations are integrated twice to produce displacements
figure (1)
subplot(311)
plot(time,Accelerometer(:,1))
xlabel('Time (sec)')
ylabel('Acceleration raw (m/sec^2)')
%% Filter  Acceleration Signals
[b1 a1] = butter(order,fc,'high');
accf=filtfilt(b1,a1,Accelerometer(:,1));
subplot(412)
plot(time,accf,'r'); hold on
plot(time,Accelerometer(:,1),'b')
xlabel('Time (sec)')
ylabel('Acceleration filtered (m/sec^2)')
%% First Integration (Acceleration - Veloicty)
velocity=cumtrapz(time,Accelerometer(:,1));
subplot(312)
plot(time,velocity)
xlabel('Time (sec)')
ylabel('Velocity (m/sec)')
%% Filter  Veloicty Signals
[b2 a2] = butter(order,fc,'high');
velf = filtfilt(b2,a2,velocity);
%% Second Integration   (Velocity - Displacement)
Displacement=cumtrapz(time, velocity);
subplot(313)
plot(time,Displacement)
xlabel('Time (sec)')
ylabel('Displacement (m)')
%===================================================================%
%% Gyroscope integration 
figure (2)
subplot(311)
plot(time,Gyroscope(:,3))
xlabel('Time (sec)')
ylabel('Gyro raw (rad/sec)')

%% Filter  Gyros Signals
[b3 a3] = butter(order,fc,'high');
gyf=filtfilt(b3,a3,Gyroscope(:,2));
subplot(312)
plot(time,gyf,'r'); hold on
plot(time,Gyroscope(:,3),'b')
xlabel('Time (sec)')
ylabel('Gyro Filtered (rad/sec)')

%% First Integration (Gyro - Angle)
roll=cumtrapz(time, Gyroscope(:,3));
subplot(313)
plot(time,roll)
xlabel('Time (sec)')
ylabel('\psi (rad)')