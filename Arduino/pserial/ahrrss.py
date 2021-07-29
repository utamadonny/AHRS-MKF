# %%
import time
import ahrs
import csv
import numpy as np

with open('Book1.csv',mode='r',newline='',encoding='utf-8-sig') as csvfile:
    data = csv.reader(csvfile,delimiter=',')
    time1,ax,ay,az,gx,gy,gz,mx,my,mz=[],[],[],[],[],[],[],[],[],[]
    for row in data:
	    time1.append(float(row[0]))
	    ax.append(float(row[1]))
	    ay.append(float(row[2]))
	    az.append(float(row[3]))
	    gx.append(float(row[4]))
	    gy.append(float(row[5]))
	    gz.append(float(row[6]))
	    mx.append(float(row[7]))
	    my.append(float(row[8]))
	    mz.append(float(row[9]))
# %%
gyro_data = np.array(list(zip(gx, gy, gz)))
acc_data  = np.array(list(zip(ax,ay,az)))
mag_data  = np.array(list(zip(mx,my,mz)))
# %%
madgwick = ahrs.filters.madgwick.Madgwick(gyr=gyro_data, acc=acc_data, mag=mag_data, dT=1/512, gain=0.1) 
# %%
print([i for i in dir(madgwick) ])
# %%