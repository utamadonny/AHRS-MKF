#%%
import time
# from turtle import color
# from matplotlib import projections
# from matplotlib.pyplot import figure
import serial
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

ser=serial.Serial('COM4',115200)

#%% 
# while True:
# for i in range(50):
# read=ser.readline()
# decode=read.decode()
# string=decode.strip()
#%%
# or0,rlb,ptb,ywb,or1,gxb,gyb,gzb,or2,l1b,l2b,l3b,or3,mxb,myb,mzb,or4,axb,ayb,azb,orr,vxb,vyb,vzb,or5,sys,or6,gys,or7,acs,or8,mgs,axm,aym,azm,gxm,gym,gzm,mxm,mym,mzm,dummy=string.split(',',41)
# time.sleep(1)
# type(string)
#%%
dmxb= []
dmyb= []
dmzb= []
dmzm= []
dmym= []
dmxm= []
for i in range(2000):
	read=ser.readline()
	if read :
		decode=read.decode()
		string=decode.strip()
		split=string.split(',')
		# rlb,ptb,ywb,gxb,gyb,gzb,l1b,l2b,l3b,mxb,myb,mzb,axb,ayb,azb,vxb,vyb,vzb,sys,gys,acs,mgs,axm,aym,azm,gxm,gym,gzm,mxm,mym,mzm=string.split(',')
		rlb,ptb,ywb,gxb,gyb,gzb,\
		l1b,l2b,l3b,mxb,myb,mzb,\
		axb,ayb,azb,vxb,vyb,vzb,\
		sys,gys,acs,mgs,axm,aym,\
		azm,gxm,gym,gzm,mxm,mym,mzm\
		=[float(x) for x in string.split(',')]
		# if sys == 3 and mgs == 3:
		print(split)
		dmxb.append(mxb)
		# print(myb)
		dmyb.append(myb)
		# print(mzb)
		dmzb.append(mzb)
		# print(mxm)
		dmxm.append(mxm)
		# print(mym)
		dmym.append(mym)
		# print(mzm)
		dmzm.append(mzm)
ser.close()

# fig=plt.figure()
# # ax=plt.axes(projections="3d")
# ax = fig.add_subplot(projection='3d')
# # for m, zlow, zhigh in [('o', -50, -25), ('^', -30, -5)]:
# #     ax.scatter(mxb, myb, mzb, marker=m)
# ax.scatter3D(mxb,myb,mzb)

plt.title("graph")
# plt.show()
# plt.show()
fig = plt.figure(figsize = (10, 7))
# ay = plt.axes(projections="3d")
ax = plt.axes(projection ="3d")
# ax1 = plt.axes(projection ="3d")

# # Creating plot
ax.scatter3D(dmxb, dmyb, dmzb, color = "green")
ax.scatter3D(dmxm,dmym,dmzm, color="blue")
plt.title("simple 3D scatter plot")
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
#
# show plot
# plt.plot(dmxb)
plt.show()


# %%
