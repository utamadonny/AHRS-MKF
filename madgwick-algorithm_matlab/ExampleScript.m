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
% when theta approaches �90 degrees. This due to a singularity in the Euler
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
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal
% set(0,'DefaultFigureWindowStyle','docked')
% set(0,'DefaultFigureWindowStyle','normal')
%% Debug#0 export csv test
% file = 'F:\Git Ubuntu\BelajarJulia\ahrs\2d250v1.csv';
% Time = readmatrix(file ,'range','A:A','OutputType','datetime');
% reuler = readmatrix(file ,'range','B:D','OutputType','double');
% Accelerometer = readmatrix(file ,'range','E:G','OutputType','double');
% Gyroscope = readmatrix(file ,'range','H:J','OutputType','double');
% Magnetometer = readmatrix(file ,'range','K:M','OutputType','double');
% aeuler = readmatrix(file ,'range','N:P','OutputType','double');
% time = readmatrix(file ,'range','Y:Y','OutputType','double');
%% (1a) Import and plot sensor data
% load("Nilai Beta");          |       RMSE             | Learning
                          %Beta| Yaw    | Pitch | Roll  | Time
% load('Data2/5d80v.mat');    %  5 | 1.7704 |0.1025 |0.2390 | 5.1
% load('Data/2v.mat');    %  2 | 0.5802 |0.1795 |0.1252 | 21
% load('Data/2v0.mat');   %  2 | 0.6981	|0.1878 |0.1182 | 2.329
% load('Data/2v1.mat');   %  2 | 0.6615	|0.1509 |0.1189 | 2.5
% load('Data/2v2.mat');   %  2 | 0.8225 |0.1575 |0.1158 | 2.3

% load('Data/1v0.mat');   %  1 | 0.7743	|0.1031 |0.0901 | 20 
% load('Data/1v1.mat');   %  1 | 0.7632	|0.0960 |0.0868 | 22
% load('Data/1v2.mat');   %  1 | 0.4812	|0.0657 |0.0917 | 20
% load('Data/1v3.mat');   %  1 | 0.2862	|0.1743 |0.2434 | 37
% load('Data/1v4.mat');   %  1 | 0.2748	|0.0548 |0.1821 | 30
% load('Data/1v5.mat');   %  1 | 0.3933	|0.0945 |0.0608 | 10
% load('Data/0c5v.mat');  % 0.5| 1.0921 |0.0570 |0.0852 | 52
% load('Data/0c5v0.mat'); % 0.5| 0.3862 |0.1387 |0.0483 | 53
% load('Data/0c5v1.mat'); % 0.5| 0.5399 |0.1218 |0.0762 | 49
% load('Data/0c5v2.mat'); % 0.5| 0.9983 |0.1726 |0.0542 | 65
% load('Data/0c5v3.mat'); % 0.5| 0.7166 |0.0854 |0.0533 | 10
% load('Data/0c5v4.mat'); % 0.5| 0.2174 |0.1159 |0.0382 | 72
% load('Data2/0c5d10v.mat');     % 0.5| 0.5318 |0.1283 |0.0297 | 96.216
% load('Data2/0c5d125v.mat');    %  1 | 0.6149 |0.2388 |0.2217 | 29.282
% load('Data/0c3v.mat');  % 0.3| 1.1805 |0.3711 |0.0646 | 108
% load('Data/0c3v0.mat'); % 0.3| 0.3291 |0.1873 |0.1282 | 109
% load('Data/0c3v1.mat'); % 0.3| 0.2668 |0.3145 |0.3635 | 130
% load('Data/0c3v2.mat'); % 0.3| 0.1829 |0.2612 |0.2934 | 131
% load('Data/0c3v3.mat'); % 0.3| 0.3759 |0.3245 |0.4334 | 116.982
% load('Data/0c3v4.mat'); % 0.3| 0.2295 |0.0993 |0.2221 | 117 
% load('Data/0c3v5.mat'); % 0.3| 1.0563 |0.3481 |0.0790 | 112
% load('Data2/0c1d270v.mat');    % 0.1| 0.5294 |0.2695 |0.0706 | 256.58
% load('Data/0c1v0.mat'); % 0.1| 0.0610 |0.1356 |0.0579 | 416

% load('Data/2021-09-08 17-19-10'); %Beta = 0.05 ga dapet
% load('Data/2021-09-08 15-58-12'); %Beta = 0.01 ga dapet kurang data
% load('Data/2021-07-08 22-03-03.mat'); % salah, Hz =100
% load('Data/2021-07-16 21-43-46.mat'); % yang katanya ada delay, Hz=100 Beta=0.1
% load('Data/1v0.mat');   %  1 | 0.3006	|0.2033 |0.3064 | 5

% KESIMPULAN : Learning time vary karena derajat akhir/goal nya
%% (1b) Import and plot sensor data  (Semua di 100derajat yaw)
% load("Nilai Beta");           |       RMSE             | Learningx`x``
                               %Beta | Yaw    | Pitch | Roll  | Time
Beta='0c1';
X=string(Beta)+'d270v';
load ('Data2/'+string(X)+'.mat')
% load('Data2/5d100v.mat');    %  5 | 0.7674 |0.1086 |0.2211 |   4.28
% load('Data2/5d100v0.mat');   %  5 | 0.8946 |0.1176 |0.2417 |   5.7 
% load('Data2/5d100v1.mat');   %  5 | 0.8399 |0.1244 |0.2547 |   4.3

% load('Data2/5d150v.mat');    %  5 | 0.4237 |0.2530 |0.2043 |   1.441
% load('Data2/5d150v0.mat');   %  5 | 0.5498 |0.2815 |0.2315 |   1.5
% load('Data2/5d150v1.mat');   %  5 | 0.7935 |0.2894 |0.1823 |   1.6

% load('Data2/5d250v.mat');    %  5 | 1.9919 |0.1518 |0.2826 |   3.816
% load('Data2/5d250v0.mat');   %  5 | 0.7389 |0.1488 |0.2761 |   3.882
% load('Data2/5d250v1.mat');   %  5 | 0.7395 |0.1460 |0.2780 |   3.6

% load('Data2/2d100v.mat');    %  2 | 1.1868 |0.1599 |0.2730 |  10.583
% load('Data2/2d100v0.mat');   %  2 | 1.2675 |0.1645 |0.2739 |  10.528
% load('Data2/2d100v1.mat');   %  2 | 1.0851 |0.1694 |0.2712 |  10.62

% load('Data2/2d150v.mat');    %  2 | 0.4700 |0.1554 |0.1181 |   3.822
% load('Data2/2d150v0.mat');   %  2 | 0.5647 |0.1569 |0.1208 |   3.731
% load('Data2/2d150v1.mat');   %  2 | 0.6189 |0.1543 |0.1448 |   3.757

% load('Data2/2d250v.mat');    %  2 | 1.1767 |0.0805 |0.1504 |   9.486
% load('Data2/2d250v0.mat');   %  2 | 1.4208 |0.0830 |0.1464 |   9.444
% load('Data2/2d250v1.mat');   %  2 | 1.2257 |0.0909 |0.1450 |   9.502

% load('Data2/1d100v.mat');    %  1 | 0.8913 |0.0592 |0.1051 |  21.11
% load('Data2/1d100v0.mat');   %  1 | 0.5818 |0.0711 |0.1401 |  21.146
% load('Data2/1d100v1.mat');   %  1 | 0.8914 |0.0957 |0.1702 |  20.89

% load('Data2/1d150v.mat');    %  1 | 0.7892 |0.0800 |0.0730 |   7.15
% load('Data2/1d150v0.mat');   %  1 | 0.7831 |0.0811 |0.0676 |   7.377
% load('Data2/1d150v1.mat');   %  1 | 0.6479 |0.0800 |0.0650 |   7.042

% load('Data2/1d250v.mat');    %  1 | 0.3262 |0.1179 |0.1106 |  20.63
% load('Data2/1d250v0.mat');   %  1 | 0.2914 |0.1003 |0.1037 |  19.758
% load('Data2/1d250v1.mat');   %  1 | 0.3176 |0.1136 |0.1057 |  19.644

% load('Data2/0c5d100v.mat');  % 0.5| 0.5620 |0.0572 |0.0856 |  41
% load('Data2/0c5d100v0.mat'); % 0.5| 0.3617 |0.0797 |0.1247 |  41.092
% load('Data2/0c5d100v1.mat'); % 0.5| 0.6390 |0.0830 |0.1108 |  41.116

% load('Data2/0c5d150v.mat');  % 0.5| 0.2675 |0.0607 |0.0615 |  15.167
% load('Data2/0c5d150v0.mat'); % 0.5| 0.2796 |0.0540 |0.0423 |  14.562
% load('Data2/0c5d150v1.mat'); % 0.5| 0.2223 |0.0503 |0.0391 |  14.963

% load('Data2/0c5d250v.mat');  % 0.5| 0.5192 |0.0713 |0.0618 |  37.387
% load('Data2/0c5d250v0.mat'); % 0.5| 0.2823 |0.0837 |0.0599 |  37.901
% load('Data2/0c5d250v1.mat'); % 0.5| 0.3649 |0.0772 |0.0591 |  36

% load('Data2/0c3d100v.mat');  % 0.3| 1.1805 |0.3711 |0.0646 | 
% load('Data2/0c3d100v0.mat'); % 0.3| 0.3291 |0.1873 |0.1282 | 
% load('Data2/0c3d100v1.mat'); % 0.3| 0.2668 |0.3145 |0.3635 | 

% load('Data2/0c1d100v.mat');  % 0.1| 0.8591 |0.0529 |0.0560 | 211.916
% load('Data2/0c1d100v0.mat'); % 0.1| 1.2564 |0.0499 |0.1374 | 212.556
% load('Data2/0c1d100v1.mat'); % 0.1| 1.1304 |0.0461 |0.1239 | 211.612
%% Set Parameter
A='Time (s) \beta='+string(Beta)+' Data2='+string(X);
%% Set Parameter
% sta = 800; stb = length(time);
% SamplePeriode = 100;
% BetaQ= 0.5;
% save('Data\aaaaa.mat');
% Kp= 10; Ki=0;
%% Debug#1
% Note: beta arduino = 5 ==> beta matlab =0.5
% Gyroscope = ld.sensorData.AngularVelocity;
% Accelerometer = ld.sensorData.Acceleration;
% Magnetometer=ld.sensorData.MagneticField;
% time= 1:1:1600;
% Fs=200;
% time = (0:decim:size(Accelerometer,1)-1)/Fs;
%% Debug#2
% dat1 = load('Data2/2v.mat');    %  2 | 1.1868 |0.1599 |0.2730 | 12
% dat2 = load('Data2/1v.mat');    %  1 | 0.8913 |0.0592 |0.1051 | 22
% dat3 = load('Data2/0c5v.mat');  % 0.5| 0.5620 |0.0572 |0.0856 | 41
% figure('Name', 'Euler Angles');
% hold on;
% plot(dat1.time, dat1.euler(:,1), 'b'); %yaw
% % plot(dat1.time, dat1.aeuler(:,1)); %yaw
% plot(dat1.time, dat1.reuler(:,1)); %yaw
% plot(dat2.time, dat2.euler(:,1), 'b'); %yaw
% % plot(dat2.time, dat2.aeuler(:,1)); %yaw
% plot(dat2.time, dat2.reuler(:,1)); %yaw
% plot(dat3.time, dat3.euler(:,1), 'b'); %yaw
% % plot(dat3.time, dat3.aeuler(:,1)); %yaw
% plot(dat3.time, dat3.reuler(:,1)); %yaw
% title('Euler angles');
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% % legend('\phi', '\theta', '\psi')
% % legend('\phi', '\theta', '\psi','a\phi', 'a\theta', 'a\psi');
% % legend('matlab\phi', 'matlab\theta', 'matlab\psi'...
% %     ,'arduino\phi', 'arduino\theta', 'arduino\psi'...
% %     ,'real\phi', 'real\theta', 'real\psi');
% hold off;
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
savefig('Image/input/'+string(X)+'.fig');
% % fig2plotly()

%% (4a) Process sensor data through algorithm
% SamplePeriode = 100;
% BetaQ= 0.05;
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
% unreliable when the middle angles of the sequence (theta) approaches �90
% degrees. This problem c ommonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
% euler = quatern2euler(quaternConj(quaternion));
offset = 180;
euler(:,3)=euler(:,3)+offset; % yaw membutuhkan offset
euler=tuker(euler,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
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
xlabel(A);
ylabel('Angle (deg)');
% legend('\phi', '\theta', '\psi')
% legend('\phi', '\theta', '\psi','a\phi', 'a\theta', 'a\psi');
legend('matlab\phi', 'matlab\theta', 'matlab\psi'...
    ,'arduino\phi', 'arduino\theta', 'arduino\psi'...
    ,'real\phi', 'real\theta', 'real\psi');
hold off;
% fig2plotly()

%% End of script
% sta = 1167; stb = length(time);
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

% max(euler(sta:length(euler),1))  % For Scale Max range

%% Save to CSV
% save('dummy.mat');
% fd= load('dummy.mat');
% T1=array2table(fd.euler);
% T1.Properties.VariableNames(1:3) = {'myaw','mpitch','mroll'};
% T2=array2table(fd.reuler);
% T2.Properties.VariableNames(1:3) = {'yaw','pitch','roll'};
% T3=array2table(fd.aeuler);
% T3.Properties.VariableNames(1:3) = {'ayaw','apitch','aroll'};
% timet=array2table(time);
% % writematrix( 
% Nums=[T2 T3 T1 timet];
% writetable(Nums,'2021-07-02 08-47-47M1.csv');
% % Nums=[reuler meuler euler time];
% % writematrix(Nums,'dummy.csv')
% % csvwrite('2021-07-16 21-43-46M.csv',filedata.reuler,filedata.meuler,filedata.euler)
% % save('Data/0c3v.mat');
%%
