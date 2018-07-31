# -*- coding: utf-8 -*-
"""
Created on Fri Jul 13 20:24:13 2018

@author: addiewan
"""

import pandas as pd
import matplotlib.pyplot as plt
from glob import glob
import os
cwd = os.getcwd()

deg_per_step = 1.8/(72*500)

data_locs = glob(cwd +'/*.csv')
data = {}
test_vels=[]
for file in data_locs:
    test_vel=file.split('.csv')[0].split('_vel_')[-1]
    test_vels.append(test_vel)
    data[test_vel] = pd.read_csv(file)
    hz=1./data[test_vel].elapsed.diff().mean()
    t0=0
    t1=data[test_vel].elapsed[0]

#Compute the average velocites after the step response settles out
mask_settle = data['15'].elapsed > 2.5
for run in test_vels:
    data['settle_vel_'+str(run)] = data[str(run)].loc[mask_settle,'imu_ang_z'].mean()
data['settle_ratios']=[data['settle_vel_80000']/data['settle_vel_15'],
                        data['settle_vel_80000']/data['settle_vel_15000']*15000,
                        data['settle_vel_80000']/data['settle_vel_2500']*2500,
                        data['settle_vel_80000']/data['settle_vel_500']*500,
                        data['settle_vel_80000']/data['settle_vel_100']*100,
                        data['settle_vel_80000']/data['settle_vel_15']*15]      

#Compute scaling factor to get from IMU velocity reading to deg/sec
data['settle_vel_act']=[]
data['deg_per_sec_cmd']=[]
data['deg_per_sec_scale']=[]
for i in range(len(test_vels)):
    data['settle_vel_act'].append(int(test_vels[i]))
    data['deg_per_sec_cmd'].append(deg_per_step*int(test_vels[i]))
    data['deg_per_sec_scale'].append(data['deg_per_sec_cmd'][-1]/data['settle_vel_'+test_vels[i]])
    
#Looks like imu scaling factor is ~14
imu2deg_sec = 14
cnt=0
for run in test_vels:
    plt.figure(2)
    x=data[str(run)]['elapsed']
    y1=data[str(run)]['imu_ang_z']*imu2deg_sec
    #y2=data[str(run)]['imu_ang_z'].rolling(1000,center=True).mean()#*180./np.pi 
    plt.plot(x,y1,'-o',label='imu_ang_z for ptu cmd= '+str(run)+' pos/sec')
    plt.figure(int(run))
    plt.plot(x,y1,'-o',label='imu_ang_z for ptu cmd= '+str(run)+' pos/sec')
    plt.title('IMU data: sampling rate='+str(hz)+'hz\n sampling time='+str(round(t1-t0,4))+'seconds\n'+str(data['deg_per_sec_cmd'][cnt])+' deg/sec')
    plt.legend()
    cnt+=1
    #plt.plot(x,y2,'-o',label='imu_ang_z rolling for ptu cmd= '+str(run)+' pos/sec')
plt.figure(2)
plt.title('IMU data: sampling rate='+str(hz)+'hz\n sampling time='+str(round(t1-t0,4))+'seconds')
plt.legend()

 


