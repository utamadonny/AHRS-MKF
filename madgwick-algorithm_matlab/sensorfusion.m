clear a;
a = arduino('COM4', 'Uno', 'Libraries', 'I2C');
%% 
imu = mpu9250(a)
fs = 100; % Sample Rate in Hz   
% imu = mpu6050(a,'SampleRate',fs,'OutputFormat','matrix','I2CAddress',{'0x68','0x0C'});