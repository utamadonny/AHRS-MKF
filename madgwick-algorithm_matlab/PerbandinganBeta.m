%% Start of script

addpath('quaternion_library');      % include quaternion library
% addpath('Data');                  % add data
%% Clear 
close all;                          % close all figures
% clear;                              % clear all variables
% clc;                                % clear the command terminal
% set(0,'DefaultFigureWindowStyle','docked')
% set(0,'DefaultFigureWindowStyle','normal')
%% FILENAME
Beta='Perbandingan Beta';
RPY='';
X=string(Beta)+''+string(RPY)+'v';
% load ('Data2/'+string(X)+'.mat')
A='Time (s) \beta='+string(Beta)+' Data2='+string(X);

SamplePeriode = 100;

%% (1b) Import and plot sensor data  (Semua di 100derajat yaw)
% load("Nilai Beta");           |       RMSE             | Learningx`x``
                               %Beta | Yaw    | Pitch | Roll  | Time

a1 = load('Data2/5d100v.mat');    %  5 | 0.7674 |0.1086 |0.2211 |   4.28
% load('Data2/5d100v0.mat');   %  5 | 0.8946 |0.1176 |0.2417 |   5.7 
% load('Data2/5d100v1.mat');   %  5 | 0.8399 |0.1244 |0.2547 |   4.3

a2 = load('Data2/5d150v.mat');    %  5 | 0.4237 |0.2530 |0.2043 |a   1.441
% load('Data2/5d150v0.mat');   %  5 | 0.5498 |0.2815 |0.2315 |   1.5
% load('Data2/5d150v1.mat');   %  5 | 0.7935 |0.2894 |0.1823 |   1.6

a3 = load('Data2/5d250v.mat');    %  5 | 1.9919 |0.1518 |0.2826 |aa   3.816
% load('Data2/5d250v0.mat');   %  5 | 0.7389 |0.1488 |0.2761 |   3.882
% load('Data2/5d250v1.mat');   %  5 | 0.7395 |0.1460 |0.2780 |   3.6

a4 = load('Data2/2d100v.mat');    %  2 | 1.1868 |0.1599 |0.2730 |  10.583
% load('Data2/2d100v0.mat');   %  2 | 1.2675 |0.1645 |0.2739 |  10.528
% load('Data2/2d100v1.mat');   %  2 | 1.0851 |0.1694 |0.2712 |  10.62

a5 = load('Data2/2d150v.mat');    %  2 | 0.4700 |0.1554 |0.1181 |   3.822
% load('Data2/2d150v0.mat');   %  2 | 0.5647 |0.1569 |0.1208 |   3.731
% load('Data2/2d150v1.mat');   %  2 | 0.6189 |0.1543 |0.1448 |   3.757

a6 = load('Data2/2d250v.mat');    %  2 | 1.1767 |0.0805 |0.1504 |   9.486
% load('Data2/2d250v0.mat');   %  2 | 1.4208 |0.0830 |0.1464 |   9.444
% load('Data2/2d250v1.mat');   %  2 | 1.2257 |0.0909 |0.1450 |   9.502

a7 = load('Data2/1d100v.mat');    %  1 | 0.8913 |0.0592 |0.1051 |  21.11
% load('Data2/1d100v0.mat');   %  1 | 0.5818 |0.0711 |0.1401 |  21.146
% load('Data2/1d100v1.mat');   %  1 | 0.8914 |0.0957 |0.1702 |  20.89

a8 = load('Data2/1d150v.mat');    %  1 | 0.7892 |0.0800 |0.0730 |   7.15
% load('Data2/1d150v0.mat');   %  1 | 0.7831 |0.0811 |0.0676 |   7.377
% load('Data2/1d150v1.mat');   %  1 | 0.6479 |0.0800 |0.0650 |   7.042

a9 = load('Data2/1d250v.mat');    %  1 | 0.3262 |0.1179 |0.1106 |  20.63
% load('Data2/1d250v0.mat');   %  1 | 0.2914 |0.1003 |0.1037 |  19.758
% load('Data2/1d250v1.mat');   %  1 | 0.3176 |0.1136 |0.1057 |  19.644

a10 = load('Data2/0c5d100v.mat');  % 0.5| 0.5620 |0.0572 |0.0856 |  41
% load('Data2/0c5d100v0.mat'); % 0.5| 0.3617 |0.0797 |0.1247 |  41.092
% load('Data2/0c5d100v1.mat'); % 0.5| 0.6390 |0.0830 |0.1108 |  41.116

a11 = load('Data2/0c5d150v.mat');  % 0.5| 0.2675 |0.0607 |0.0615 |  15.167
% load('Data2/0c5d150v0.mat'); % 0.5| 0.2796 |0.0540 |0.0423 |  14.562
% load('Data2/0c5d150v1.mat'); % 0.5| 0.2223 |0.0503 |0.0391 |  14.963

a12 = load('Data2/0c5d250v.mat');  % 0.5| 0.5192 |0.0713 |0.0618 |  37.387
% load('Data2/0c5d250v0.mat'); % 0.5| 0.2823 |0.0837 |0.0599 |  37.901
% load('Data2/0c5d250v1.mat'); % 0.5| 0.3649 |0.0772 |0.0591 |  36

a13 = load('Data2/0c3d50v0.mat');  % 0.3| 1.1805 |0.3711 |0.0646 | 
% load('Data2/0c3d100v0.mat'); % 0.3| 0.3291 |0.1873 |0.1282 | 
% load('Data2/0c3d100v1.mat'); % 0.3| 0.2668 |0.3145 |0.3635 | 

% load('Data2/0c3d50v.mat'); % 0.3| 0.3759 |0.3245 |0.4334 | 116.982
% load('Data2/0c3d50v0.mat'); % 0.3| 0.2295 |0.0993 |0.2221 | 117 

a14 =  load('Data2/0c3d60v.mat');  % 0.3| 1.1805 |0.3711 |0.0646 | 108
% load('Data2/0c3d60v0.mat'); % 0.3| 1.0563 |0.3481 |0.0790 | 112

a15 = load('Data2/0c1d100v.mat');  % 0.1| 0.8591 |0.0529 |0.0560 | 211.916
% load('Data2/0c1d100v0.mat'); % 0.1| 1.2564 |0.0499 |0.1374 | 212.556
% load('Data2/0c1d100v1.mat'); % 0.1| 1.1304 |0.0461 |0.1239 | 211.612

a16 = load('Data2/0c1d270v.mat');    % 0.1| 0.5294 |0.2695 |0.0706 | 256.58

a17 = load('Data2/0c5d10v.mat');  % 0.5| 0.5318 |0.1283 |0.0297 | 96.216
a18 = load('Data2/0c5d125v.mat'); %  1 | 0.6149 |0.2388 |0.2217 | 29.282
a19 = load('Data2/5d80v.mat');    %  5 | 1.7704 |0.1025 |0.2390 | 5.1

%% 
figure('Name', 'Euler Angles');
hold on;

% % % % %%%%%%%%%%%ABSTRACT%%%%%%%%%%%%%%%%%%%
% plot(time, euler(:,1)); %yaw 
% plot(time, aeuler(:,1)); %yaw
% plot(time, reuler(:,1)); %yaw 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot(a1.time, a1.euler(:,1) ,'b' ,'DisplayName', 'Beta=5'); %yaw 
% plot(a1.time, a1.aeuler(:,1),'b' ,'DisplayName', 'Beta=5'); %yaw
plot(a1.time, a1.reuler(:,1),'b' ,'DisplayName', 'Beta=5'); %yaw  

plot(a2.time, a2.euler(:,1) ,'b','DisplayName', 'Beta=5'); %yaw 
% plot(a2.time, a2.aeuler(:,1),'b','DisplayName', 'Beta=5'); %yaw
plot(a2.time, a2.reuler(:,1),'b','DisplayName', 'Beta=5'); %yaw 

plot(a3.time, a3.euler(:,1) ,'b','DisplayName', 'Beta=5'); %yaw 
% plot(a3.time, a3.aeuler(:,1),'b','DisplayName', 'Beta=5'); %yaw
plot(a3.time, a3.reuler(:,1),'b','DisplayName', 'Beta=5'); %yaw 

plot(a4.time, a4.euler(:,1) ,'r','DisplayName', 'Beta=2'); %yaw 
% plot(a4.time, a4.aeuler(:,1),'r','DisplayName', 'Beta=2'); %yaw
plot(a4.time, a4.reuler(:,1),'r','DisplayName', 'Beta=2'); %yaw 

plot(a5.time, a5.euler(:,1) ,'r','DisplayName', 'Beta=2'); %yaw 
% plot(a5.time, a5.aeuler(:,1),'r','DisplayName', 'Beta=2'); %yaw
plot(a5.time, a5.reuler(:,1),'r','DisplayName', 'Beta=2'); %yaw 

plot(a6.time, a6.euler(:,1) ,'r','DisplayName', 'Beta=2'); %yaw 
% plot(a6.time, a6.aeuler(:,1),'r','DisplayName', 'Beta=2'); %yaw
plot(a6.time, a6.reuler(:,1),'r','DisplayName', 'Beta=2'); %yaw 

plot(a7.time, a7.euler(:,1) ,'m','DisplayName', 'Beta=1'); %yaw 
% plot(a7.time, a7.aeuler(:,1),'m','DisplayName', 'Beta=1'); %yaw
plot(a7.time, a7.reuler(:,1),'m','DisplayName', 'Beta=1'); %yaw 

plot(a8.time, a8.euler(:,1) ,'m','DisplayName', 'Beta=1'); %yaw 
% plot(a8.time, a8.aeuler(:,1),'m','DisplayName', 'Beta=1'); %yaw
plot(a8.time, a8.reuler(:,1),'m','DisplayName', 'Beta=1'); %yaw 

plot(a9.time, a9.euler(:,1) ,'m','DisplayName', 'Beta=1'); %yaw 
% plot(a9.time, a9.aeuler(:,1),'m','DisplayName', 'Beta=1'); %yaw
plot(a9.time, a9.reuler(:,1),'m','DisplayName', 'Beta=1'); %yaw 

plot(a10.time, a10.euler(:,1) ,'c','DisplayName', 'Beta=0.5'); %yaw 
% plot(a10.time, a10.aeuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw
plot(a10.time, a10.reuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw 

plot(a11.time, a11.euler(:,1) ,'c','DisplayName', 'Beta=0.5'); %yaw 
% plot(a11.time, a11.aeuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw
plot(a11.time, a11.reuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw 

plot(a12.time, a12.euler(:,1), 'c','DisplayName', 'Beta=0.5'); %yaw 
% plot(a12.time, a12.aeuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw
plot(a12.time, a12.reuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw 

plot(a13.time, a13.euler(:,1) ,'Color',[0.4940 0.1840 0.5560],'DisplayName', 'Beta=0.3'); %yaw 
% plot(a13.time, a13.aeuler(:,1),'Color',[0.4940 0.1840 0.5560],'DisplayName', 'Beta=0.3'); %yaw
plot(a13.time, a13.reuler(:,1),'Color',[0.4940 0.1840 0.5560],'DisplayName', 'Beta=0.3'); %yaw 

plot(a14.time, a14.euler(:,1) ,'Color',[0.4940 0.1840 0.5560],'DisplayName', 'Beta=0.3'); %yaw 
% plot(a14.time, a14.aeuler(:,1),'Color',[0.4940 0.1840 0.5560],'DisplayName', 'Beta=0.3'); %yaw
plot(a14.time, a14.reuler(:,1),'Color',[0.4940 0.1840 0.5560],'DisplayName', 'Beta=0.3'); %yaw 

plot(a15.time, a15.euler(:,1) ,'g','DisplayName', 'Beta=0.1'); %yaw 
% plot(a15.time, a15.aeuler(:,1),'g','DisplayName', 'Beta=0.1'); %yaw
plot(a15.time, a15.reuler(:,1),'g','DisplayName', 'Beta=0.1'); %yaw 

plot(a16.time, a16.euler(:,1) ,'g','DisplayName', 'Beta=0.1'); %yaw 
% plot(a16.time, a16.aeuler(:,1),'g','DisplayName', 'Beta=0.1'); %yaw
plot(a16.time, a16.reuler(:,1),'g','DisplayName', 'Beta=0.1'); %yaw 

plot(a17.time, a17.euler(:,1) ,'c','DisplayName', 'Beta=0.5'); %yaw 
% plot(a17.time, a17.aeuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw
plot(a17.time, a17.reuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw 

plot(a18.time, a18.euler(:,1) ,'c','DisplayName', 'Beta=0.5'); %yaw 
% plot(a18.time, a18.aeuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw
plot(a18.time, a18.reuler(:,1),'c','DisplayName', 'Beta=0.5'); %yaw 

plot(a19.time, a19.euler(:,1) ,'b' ,'DisplayName', 'Beta=5'); %yaw 
% plot(a19.time, a19.aeuler(:,1),'b' ,'DisplayName', 'Beta=5'); %yaw
plot(a19.time, a19.reuler(:,1),'b' ,'DisplayName', 'Beta=5'); %yaw  


xlabel(A);
ylabel('Angle (deg)');
legend(legendUnq());
hold off;