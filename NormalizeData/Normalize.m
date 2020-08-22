clc; clear;
 
data = xlsread('D:\DONNY\Documents\AHRS-MKF\Data\Steady IMU\IMUdiamDonny.xlsx',1,'E6:P16');
 
% Proses Normalisasi Data
max_data = max(max(data));
min_data = min(min(data));
 
[m,n] = size(data);
data_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        data_norm(x,y) = 0.1+0.8*(data(x,y)-min_data)/(max_data-min_data);
    end
end