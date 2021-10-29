%% Mukaddimah
clear
clc
%% Import Data
file = 'F:\Git Ubuntu\BelajarJulia\ahrs\2d250v1.csv';
Time = readmatrix(file ,'range','A:A','OutputType','datetime');
reuler = readmatrix(file ,'range','B:D','OutputType','double');
Accelerometer = readmatrix(file ,'range','E:G','OutputType','double');
Gyroscope = readmatrix(file ,'range','H:J','OutputType','double');
Magnetometer = readmatrix(file ,'range','K:M','OutputType','double');
aeuler = readmatrix(file ,'range','N:P','OutputType','double');
time = readmatrix(file ,'range','Y:Y','OutputType','double');

%% Create Algorithm
SampleRate = 100;
decim = 2;
numSamples = size(Accelerometer,1);
time = (0:decim:(numSamples-1))'/SampleRate;
FUSE = ahrsfilter('SampleRate',SampleRate,'DecimationFactor',decim,'NED','RF');

%%
orientation = FUSE(Accelerometer,Gyroscope,Magnetometer);

orientationEulerAngles = eulerd(orientation,'ZYX','frame');

figure(1)
plot(time,orientationEulerAngles(:,1), ...
     time,orientationEulerAngles(:,2), ...
     time,orientationEulerAngles(:,3))
xlabel('Time (s)')
ylabel('Rotation (degrees)')
legend('z-axis','y-axis','x-axis')
title('Filtered IMU Data')
%%
time = (0:(numSamples-1))'/SampleRate;
figure(1)
subplot(3,1,1)
plot(time,Accelerometer)
title('Accelerometer Reading')
ylabel('Acceleration (m/s^2)')

subplot(3,1,2)
plot(time,Magnetometer)
title('Magnetometer Reading')
ylabel('Magnetic Field (\muT)')

subplot(3,1,3)
plot(time,Gyroscope)
title('Gyroscope Reading')
ylabel('Angular Velocity (rad/s)')
xlabel('Time (s)')
%%
