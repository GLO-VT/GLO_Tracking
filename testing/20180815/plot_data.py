# -*- coding: utf-8 -*-
"""
Created on Wed Aug 15 11:51:26 2018

@author: Bailey group PC
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np 

loc = r"C:\git_repos\GLO_Tracking\testing\20180815\ss_track_20180815_134056_RUN24.csv"
header = pd.read_csv(loc,nrows=1)
df = pd.read_csv(loc,skiprows=2)
ptu_pos2deg=1./(1.8/(72*500))

df['ss_time'] = df['t1']-df['t0']
df['pid_time'] = df['t2']-df['t1']
df['ptu_time'] = df['t3']-df['t2']
df['aw_time'] = df['t4']-df['t3']
df['data_time'] = df['t5']-df['t4']
df['move_neg'] = df['t7']-df['t6']
df['move_pos'] = df['t8']-df['t7']
df['stopeed'] = df['t9']-df['t8']
df['setspeed'] = df['t11']-df['t10']

x=df['elapsed']
y1=(df['imu_ang_z'])*180./np.pi
y2=df['imu_filt_x']*180./np.pi

y4=df['ss1_x_raw']

y6=df['ptu_cmd_x']
y7=df['pid_out_x']
   #y12=df['imu_filt_y']*180./np.pi

y14=df['ss1_y_raw']
 #       y5=df['ss2_x_raw']
y16=df['ptu_cmd_y'] 

#plt.figure()
#plt.title('X-Axis sensor data at '+str(header['hz'])+'hz\n kp='+str(header['kpx'])+' ki='+str(header['kix'])+' kd='+str(header['kdx']))
#plt.plot(x,y4,'o-',label='ss1_x_raw')
#plt.plot(x,y6,'o-',label='ptu cmd x')
#plt.legend()

plt.figure()
plt.plot(x,y1.rolling(10,center=True).mean(),'o',label='imu_ang_z rolling(10)')
#plt.plot(x,y1,'o-',label='imu_ang_z RAW')
plt.plot(x,y6/ptu_pos2deg,'o-',label='ptu_cmd_x')
plt.plot(x,y4,'o-',label='ss2_ss_filt_x')
#plt.plot(x,y7*10,'o',label='pid_out_x')
plt.xlabel('Time Elapsed (seconds)')
plt.ylabel('Degrees')
plt.hlines(0.1,x.values[0],x.values[-1],color='red')
plt.hlines(-0.1,x.values[0],x.values[-1],color='red')
#plt.ylim((-3,3))
#plt.title('Y-Axis sensor data at '+str(hz)+'hz\n kp='+str(params.kpy)+' ki='+str(params.kiy)+' kd='+str(params.kdy))
plt.legend()
plt.grid()

plt.figure()
plt.hist(y4,bins=400)