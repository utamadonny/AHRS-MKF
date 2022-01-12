addpath('quaternion_library');      % include quaternion library
% addpath('Data');                  % add data
%% Clear 
close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal
% set(0,'DefaultFigureWindowStyle','docked')
% set(0,'DefaultFigureWindowStyle','normal')
%% FILENAME
% load('Data2/5d100v0.mat');   %  5 | 0.8946 |0.1176 |0.2417 |   5.7 
load('Data2/1d100v.mat'); 
Beta='perbandingan';
RPY='';
X=string(Beta)+''+string(RPY)+'v';
% load ('Data2/'+string(X)+'.mat')
A='Time (s) \beta='+string(Beta)+' Data2='+string(X);
% save ('Data2/'+string(X)+'.mat')
%% Set Parameter
% sta = 800; stb = length(time);
SamplePeriode = 100;
% BetaQ= 0.1;
% save('Data\aaaaa.mat');
% Kp= 10; Ki=0;
% numSamples = size(Accelerometer,1);
% time = (0:1:(numSamples-1))'/SamplePeriode;
% save ('Data2/'+string(X)+'.mat')
%% (1b) Import and plot sensor data  (Semua di 100derajat yaw)
% load("Nilai Beta");           |       RMSE             | Learningx`x``
                               %Beta | Yaw    | Pitch | Roll  | Time

% load('Data2/5d100v.mat');    %  5 | 0.7674 |0.1086 |0.2211 |   4.28
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
xlabel(string(A)+'RAW');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel(string(A)+'RAW');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel(string(A)+'RAW');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'xy');
savefig('E:\Documents\SKRIPSI\OTW SEMHAS\GambarMatlab\Image\perbandingan\'+string(X)+'.fig');
% % fig2plotly()

%% Tes
% offset = 180;
% euler = zeros(50);
% for i = 0.1:0.1:5
%     AHRS= MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', i); % Tuning Sampleperiod and Beta
%     quaternion = zeros(length(time), 4);
%     for t = 1:length(time)
%         AHRS.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
%         quaternion(t, :) = AHRS.Quaternion;
%     end
%     
%     euler(i) = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
%     euler(i)(:,3)=euler(i)(:,3)+offset; % yaw membutuhkan offset
%     euler(i)=tuker(euler(i),1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
%     euler(i)(:,2:3)=-1*euler(i)(:,2:3); % Switch for NED 
% end
%% (4a) Process sensor data through algorithm
AHRS5 = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', 0.5); % Tuning Sampleperiod and Beta
AHRS2 = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', 0.2); % Tuning Sampleperiod and Beta
AHRS1 = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', 0.1); % Tuning Sampleperiod and Beta
AHRS0c5 = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', 0.05); % Tuning Sampleperiod and Beta
AHRS0c3 = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', 0.03); % Tuning Sampleperiod and Beta
AHRS0c1 = MadgwickAHRS('SamplePeriod', 1/SamplePeriode, 'Beta', 0.01); % Tuning Sampleperiod and Beta

quaternion5 = zeros(length(time), 4);
for t = 1:length(time)
    AHRS5.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion5(t, :) = AHRS5.Quaternion;
end

quaternion2 = zeros(length(time), 4);
for t = 1:length(time)
    AHRS2.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion2(t, :) = AHRS2.Quaternion;
end

quaternion1 = zeros(length(time), 4);
for t = 1:length(time)
    AHRS1.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion1(t, :) = AHRS1.Quaternion;
end

quaternion0c5 = zeros(length(time), 4);
for t = 1:length(time)
    AHRS0c5.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion0c5(t, :) = AHRS0c5.Quaternion;
end

quaternion0c3 = zeros(length(time), 4);
for t = 1:length(time)
    AHRS0c3.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion0c3(t, :) = AHRS0c3.Quaternion;
end

quaternion0c1 = zeros(length(time), 4);
for t = 1:length(time)
    AHRS0c1.Update(Gyroscope1(t,:)* (pi/180), Accelerometer1(t,:), Magnetometer1(t,:));% gyroscope units must be radians
    quaternion0c1(t, :) = AHRS0c1.Quaternion;
end

euler5 = quatern2euler(quaternConj(quaternion5)) * (180/pi);
euler2 = quatern2euler(quaternConj(quaternion2)) * (180/pi);
euler1 = quatern2euler(quaternConj(quaternion1)) * (180/pi);
euler0c5 = quatern2euler(quaternConj(quaternion0c5)) * (180/pi);
euler0c3 = quatern2euler(quaternConj(quaternion0c3)) * (180/pi);
euler0c1 = quatern2euler(quaternConj(quaternion0c1)) * (180/pi);% use conjugate for sensor frame relative to Earth and convert to degrees.
offset = 180;

euler5(:,3)=euler5(:,3)+offset; % yaw membutuhkan offset
euler5=tuker(euler5,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
euler5(:,2:3)=-1*euler5(:,2:3); % Switch for NED 

euler2(:,3)=euler2(:,3)+offset; % yaw membutuhkan offset
euler2=tuker(euler2,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
euler2(:,2:3)=-1*euler2(:,2:3); % Switch for NED 

euler1(:,3)=euler1(:,3)+offset; % yaw membutuhkan offset
euler1=tuker(euler1,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
euler1(:,2:3)=-1*euler1(:,2:3); % Switch for NED 

euler0c5(:,3)=euler0c5(:,3)+offset; % yaw membutuhkan offset
euler0c5=tuker(euler0c5,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
euler0c5(:,2:3)=-1*euler0c5(:,2:3); % Switch for NED 

euler0c3(:,3)=euler0c3(:,3)+offset; % yaw membutuhkan offset
euler0c3=tuker(euler0c3,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
euler0c3(:,2:3)=-1*euler0c3(:,2:3); % Switch for NED 

euler0c1(:,3)=euler0c1(:,3)+offset; % yaw membutuhkan offset
euler0c1=tuker(euler0c1,1,3); % menukar roll dan yaw, agar formatnya sama data realtime (r,p,y) ==> (y,p,r)
euler0c1(:,2:3)=-1*euler0c1(:,2:3); % Switch for NED 
%%
test=figure('Name', 'Euler Angles');
hold on;
plot(time, euler5(:,1),'DisplayName','Beta=5'); %roll
plot(time, euler2(:,1),'DisplayName','Beta=2'); %roll
plot(time, euler1(:,1),'DisplayName','Beta=1'); %roll
plot(time, euler0c5(:,1),'DisplayName','Beta=0.5'); %roll
plot(time, euler0c3(:,1),'DisplayName','Beta=0.3'); %roll
plot(time, euler0c1(:,1),'DisplayName','Beta=0.1'); %roll
plot(time, reuler(:,1),'DisplayName','Ground Truth'); %roll


xlabel(A);
ylabel('Angle (deg)');
legend()

hold off;
savefig('E:\Documents\SKRIPSI\OTW SEMHAS\GambarMatlab\Image\perbandingan\'+string(X)+'1.fig');
% fig2plotly()
%% End of script
% sta=8497;
% sta=1;
% rmsey5 = sqrt(mean((euler5(sta:stb,1) - reuler(sta:stb,1)).^2));
% rmsey2 = sqrt(mean((euler2(sta:stb,1) - reuler(sta:stb,1)).^2));
% rmsey1 = sqrt(mean((euler1(sta:stb,1) - reuler(sta:stb,1)).^2));
% rmsey0c5 = sqrt(mean((euler0c5(sta:stb,1) - reuler(sta:stb,1)).^2));
% rmsey0c3 = sqrt(mean((euler0c3(sta:stb,1) - reuler(sta:stb,1)).^2));
% rmsey0c1 = sqrt(mean((euler0c1(sta:stb,1) - reuler(sta:stb,1)).^2));
stb=length(reuler);
%% 100
% rmsey5 = sqrt(mean((euler5(182:stb,1) - reuler(182:stb,1)).^2));
% rmsey2 = sqrt(mean((euler2(445:stb,1) - reuler(445:stb,1)).^2));
% rmsey1 = sqrt(mean((euler1(840:stb,1) - reuler(840:stb,1)).^2));
% rmsey0c5 = sqrt(mean((euler0c5(1950:stb,1) - reuler(1950:stb,1)).^2));
% rmsey0c3 = sqrt(mean((euler0c3(2782:stb,1) - reuler(2782:stb,1)).^2));
% rmsey0c1 = sqrt(mean((euler0c1(8497:stb,1) - reuler(8497:stb,1)).^2));
%% 150
% rmsey5 = sqrt(mean((euler5(61:stb,1) - reuler(61:stb,1)).^2));
% rmsey2 = sqrt(mean((euler2(147:stb,1) - reuler(147:stb,1)).^2));
% rmsey1 = sqrt(mean((euler1(380:stb,1) - reuler(380:stb,1)).^2));
% rmsey0c5 = sqrt(mean((euler0c5(500:stb,1) - reuler(500:stb,1)).^2));
% rmsey0c3 = sqrt(mean((euler0c3(720:stb,1) - reuler(720:stb,1)).^2));
% rmsey0c1 = sqrt(mean((euler0c1(2939:stb,1) - reuler(2939:stb,1)).^2));
%% 250
rmsey5 = sqrt(mean((euler5(80:stb,1) - reuler(80:stb,1)).^2));
rmsey2 = sqrt(mean((euler2(486:stb,1) - reuler(486:stb,1)).^2));
rmsey1 = sqrt(mean((euler1(867:stb,1) - reuler(867:stb,1)).^2));
rmsey0c5 = sqrt(mean((euler0c5(1470:stb,1) - reuler(1470:stb,1)).^2));
rmsey0c3 = sqrt(mean((euler0c3(2554:stb,1) - reuler(2554:stb,1)).^2));
rmsey0c1 = sqrt(mean((euler0c1(7815:stb,1) - reuler(7815:stb,1)).^2));

rmsey5
rmsey2
rmsey1
rmsey0c5
rmsey0c3
rmsey0c1

%%
a100 = load('Data2/perbandingan1d100v.mat'); 
a150 = load('Data2/perbandingan1d150v.mat'); 
a250 = load('Data2/perbandingan1d250v.mat'); 

figure('Name', 'Euler Angles');
hold on;
plot(a100.time, a100.euler5(:,1),'--b','DisplayName','Beta=5'); %roll
plot(a100.time, a100.euler2(:,1),'k','DisplayName','Beta=2'); %roll
plot(a100.time, a100.euler1(:,1),':m','DisplayName','Beta=1'); %roll
plot(a100.time, a100.euler0c5(:,1),'--y','DisplayName','Beta=0.5'); %roll
plot(a100.time, a100.euler0c3(:,1),'r','DisplayName','Beta=0.3'); %roll
plot(a100.time, a100.euler0c1(:,1),'-.g','DisplayName','Beta=0.1'); %roll
plot(a100.time, a100.reuler(:,1),'DisplayName','real'); %roll

plot(a150.time, a150.euler5(:,1),'--b','DisplayName','Beta=5'); %roll
plot(a150.time, a150.euler2(:,1),'k','DisplayName','Beta=2'); %roll
plot(a150.time, a150.euler1(:,1),':m','DisplayName','Beta=1'); %roll
plot(a150.time, a150.euler0c5(:,1),'--y','DisplayName','Beta=0.5'); %roll
plot(a150.time, a150.euler0c3(:,1),'r','DisplayName','Beta=0.3'); %roll
plot(a150.time, a150.euler0c1(:,1),'-.g','DisplayName','Beta=0.1'); %roll
plot(a150.time, a150.reuler(:,1),'DisplayName','real'); %roll

plot(a250.time, a250.euler5(:,1),'--b','DisplayName','Beta=5'); %roll
plot(a250.time, a250.euler2(:,1),'k','DisplayName','Beta=2'); %roll
plot(a250.time, a250.euler1(:,1),':m','DisplayName','Beta=1'); %roll
plot(a250.time, a250.euler0c5(:,1),'--y','DisplayName','Beta=0.5'); %roll
plot(a250.time, a250.euler0c3(:,1),'r','DisplayName','Beta=0.3'); %roll
plot(a250.time, a250.euler0c1(:,1),'-.g','DisplayName','Beta=0.1'); %roll
plot(a250.time, a250.reuler(:,1),'DisplayName','real'); %roll


xlabel(A);
ylabel('Angle (deg)');
legend(legendUnq())

hold off;
savefig('E:\Documents\SKRIPSI\OTW SEMHAS\GambarMatlab\Image\perbandingan\'+string(X)+'2.fig');
%% 
figure();
beta    = [0.1 0.3 0.5 1 2 5];
rmse100 = [a100.rmsey0c1 a100.rmsey0c3 a100.rmsey0c5 a100.rmsey1 a100.rmsey2 a100.rmsey5];
rmse150 = [a150.rmsey0c1 a150.rmsey0c3 a150.rmsey0c5 a150.rmsey1 a150.rmsey2 a150.rmsey5];
rmse250 = [a250.rmsey0c1 a250.rmsey0c3 a250.rmsey0c5 a250.rmsey1 a250.rmsey2 a250.rmsey5];
hold on;
plot(beta, rmse100,'--+k','DisplayName','100 deg');
plot(beta, rmse150,':or', 'DisplayName','150 deg' );
plot(beta, rmse250,'-.*b','DisplayName','250 deg');

xlabel('\beta');
ylabel('RMSE');
legend(legendUnq())
hold off;
savefig('E:\Documents\SKRIPSI\OTW SEMHAS\GambarMatlab\Image\perbandingan\'+string(X)+'3.fig');

%%
subplot(211)
hold on;
plot(a100.time, a100.euler5(:,1),'--b','DisplayName','Beta=5'); %roll
plot(a100.time, a100.euler2(:,1),'k','DisplayName','Beta=2'); %roll
plot(a100.time, a100.euler1(:,1),':m','DisplayName','Beta=1'); %roll
plot(a100.time, a100.euler0c5(:,1),'--y','DisplayName','Beta=0.5'); %roll
plot(a100.time, a100.euler0c3(:,1),'r','DisplayName','Beta=0.3'); %roll
plot(a100.time, a100.euler0c1(:,1),'-.g','DisplayName','Beta=0.1'); %roll
plot(a100.time, a100.reuler(:,1),'DisplayName','real'); %roll

plot(a150.time, a150.euler5(:,1),'--b','DisplayName','Beta=5'); %roll
plot(a150.time, a150.euler2(:,1),'k','DisplayName','Beta=2'); %roll
plot(a150.time, a150.euler1(:,1),':m','DisplayName','Beta=1'); %roll
plot(a150.time, a150.euler0c5(:,1),'--y','DisplayName','Beta=0.5'); %roll
plot(a150.time, a150.euler0c3(:,1),'r','DisplayName','Beta=0.3'); %roll
plot(a150.time, a150.euler0c1(:,1),'-.g','DisplayName','Beta=0.1'); %roll
plot(a150.time, a150.reuler(:,1),'DisplayName','real'); %roll

plot(a250.time, a250.euler5(:,1),'--b','DisplayName','Beta=5'); %roll
plot(a250.time, a250.euler2(:,1),'k','DisplayName','Beta=2'); %roll
plot(a250.time, a250.euler1(:,1),':m','DisplayName','Beta=1'); %roll
plot(a250.time, a250.euler0c5(:,1),'--y','DisplayName','Beta=0.5'); %roll
plot(a250.time, a250.euler0c3(:,1),'r','DisplayName','Beta=0.3'); %roll
plot(a250.time, a250.euler0c1(:,1),'-.g','DisplayName','Beta=0.1'); %roll
plot(a250.time, a250.reuler(:,1),'DisplayName','real'); %roll
xlabel(A);
ylabel('Angle (deg)');
legend(legendUnq())

hold off;
subplot(212)
hold on;
plot(beta, rmse100,'--+k','DisplayName','100 deg');
plot(beta, rmse150,':or', 'DisplayName','150 deg' );
plot(beta, rmse250,'-.*b','DisplayName','250 deg');

xlabel('\beta');
ylabel('RMSE');
legend(legendUnq())
hold off;