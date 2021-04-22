clc; clear;
 
ax = xlsread('IMUdiam1.xlsx','B2:B411');
ay =xlsread('IMUdiam1.xlsx','C2:C411');
az =xlsread('IMUdiam1.xlsx','D2:D411');
Accelerometer =xlsread('IMUdiam1.xlsx',1,'B2:D411');
gx =xlsread('IMUdiam1.xlsx','E2:E411');
gy =xlsread('IMUdiam1.xlsx','F2:F411');
gz =xlsread('IMUdiam1.xlsx','G2:G411');
Gyroscope = xlsread('IMUdiam1.xlsx','E2:G411');
hx =xlsread('IMUdiam1.xlsx','H2:H411');
hy =xlsread('IMUdiam1.xlsx','I2:I411');
hz =xlsread('IMUdiam1.xlsx','J2:J411');
Magnetometer = xlsread('IMUdiam1.xlsx',1,'H2:J411');
time = xlsread('IMUdiam1.xlsx','K2:K411');
 
%% Proses Normalisasi Data per Data
max_ax = max(max(ax));
min_ax = min(min(ax));
max_ay = max(max(ay));
min_ay = min(min(ay));
max_az = max(max(az));
min_az = min(min(az));
max_gx = max(max(gx));
min_gx = min(min(gx));
max_gy = max(max(gy));
min_gy = min(min(gy));
max_gz = max(max(gz));
min_gz = min(min(gz));
max_hx = max(max(hx));
min_hx = min(min(hx));
max_hy = max(max(hy));
min_hy = min(min(hy));
max_hz = max(max(hz));
min_hz = min(min(hz));


[m,n] = size(ax);
ax_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        ax_norm(x,y) = (ax(x,y)-min_ax)/(max_ax-min_ax);
    end
end
[m,n] = size(ay);
ay_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        ay_norm(x,y) = (ay(x,y)-min_ay)/(max_ay-min_ay);
    end
end
[m,n] = size(az);
az_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        az_norm(x,y) = (az(x,y)-min_az)/(max_az-min_az);
    end
end

[m,n] = size(ax);
gx_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        gx_norm(x,y) = (gx(x,y)-min_gx)/(max_gx-min_gx);
    end
end
[m,n] = size(gy);
gy_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        gy_norm(x,y) = (gy(x,y)-min_gy)/(max_gy-min_gy);
    end
end
[m,n] = size(gz);
gz_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        gz_norm(x,y) = (gz(x,y)-min_gz)/(max_gz-min_gz);
    end
end

[m,n] = size(hx);
hx_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        hx_norm(x,y) = (hx(x,y)-min_hx)/(max_hx-min_hx);
    end
end
[m,n] = size(hy);
hy_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        hy_norm(x,y) = (hy(x,y)-min_hy)/(max_hy-min_hy);
    end
end
[m,n] = size(hz);
hz_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        hz_norm(x,y) = (hz(x,y)-min_hz)/(max_hz-min_hz);
    end
end
%% Normalisasi Semua Data
min_Accelerometer = min(min(Accelerometer));
max_Accelerometer = max(max(Accelerometer));
min_Gyroscope = min(min(Gyroscope));
max_Gyroscope = max(max(Gyroscope));
min_Magnetometer = min(min(Magnetometer));
max_Magnetometer = max(max(Magnetometer));

[m,n] = size(Accelerometer);
Accelerometer_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        Accelerometer_norm(x,y) = (Accelerometer(x,y)-min_Accelerometer)/(max_Accelerometer-min_Accelerometer);
    end
end
[m,n] = size(Magnetometer);
Magnetometer_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        Magnetometer_norm(x,y) = (Magnetometer(x,y)-min_Magnetometer)/(max_Magnetometer-min_Magnetometer);
    end
end
[m,n] = size(Gyroscope);
Gyroscope_norm = zeros(m,n);
for x = 1:m
    for y = 1:n
        Gyroscope_norm(x,y) = (Gyroscope(x,y)-min_Gyroscope)/(max_Gyroscope-min_Gyroscope);
    end
end

%% plot 
figure('Name', 'Sensor Data');
subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope Raw');
axis ([0 30 0 10]);  
hold off
subplot(3,1,2);
hold on
plot(time, gx_norm, 'r');
plot(time, gy_norm, 'g');
plot(time, gz_norm, 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope Norm /data');
axis ([0 30 0 10]);
hold off
subplot(3,1,3);
hold on
plot(time, Gyroscope_norm(:,1), 'r');
plot(time, Gyroscope_norm(:,2), 'g');
plot(time, Gyroscope_norm(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope Norm semua data');
axis ([0 30 0 10]);