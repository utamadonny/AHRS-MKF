# %%
import time
import struct
import serial
import csv

with open('Book1.csv',mode='r',newline='',encoding='utf-8-sig') as csvfile:
    data = csv.reader(csvfile,delimiter=',')
    ax,ay,az,gx,gy,gz,mx,my,mz=[],[],[],[],[],[],[],[],[]
    for row in data:
	    ax.append(float(row[0]))
	    ay.append(float(row[1]))
	    az.append(float(row[2]))
	    gx.append(float(row[3]))
	    gy.append(float(row[4]))
	    gz.append(float(row[5]))
	    mx.append(float(row[6]))
	    my.append(float(row[7]))
	    mz.append(float(row[8]))
# %%
ser=serial.Serial('/dev/ttyUSB0',9600)
time.sleep(2)
# %%
ser.write(struct.pack('>fffffffff',ax,ay,az,gx,gy,gz,mx,my,mz))
# %%
