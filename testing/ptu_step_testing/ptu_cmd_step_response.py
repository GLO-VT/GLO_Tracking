# -*- coding: utf-8 -*-
"""
Created on Fri Jul 13 15:08:29 2018

@author: Bailey group PC

Hang Test frame with ebay PTU placed on ground
IMU positioned on top/center of PTU
Multiple runs are saved sending a step re

"""
from imu import IMU
from ptu_newmark import PTU
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt
import os
cwd=os.getcwd()

ptu = PTU(com_port='COM9',baudrate=9600)
time.sleep(2)
imu = IMU(com_port='COM5',baudrate=115200)
time.sleep(0.3)
#imu.change_baudrate(921600)
#imu.imu.disconnect()
#imu = IMU(com_port='COM7',baudrate=921600)
#time.sleep(0.5)

N=1000
t0=time.time()
data={}
data['date']='20180710'
data['samples']=N
cnt=0
test_vel = [15,100,500,2500,15000,80000]

for run in test_vel:
    data[str(run)] = pd.DataFrame(columns=['imu_ang_z','elapsed'])
    t0=time.time()
    ptu_on=False
    imu_ang_z = np.zeros(N)

    for i in range(N):
        if ptu_on==False:
#            print(time.time()-t0)
            if time.time() - t0 > 2:    
                ptu_on=True
                ptu.cmd('@01SSPD'+str(test_vel[cnt])+'\r')
                print('@01SSPD'+str(test_vel[cnt])+'\r')
                ptu.cmd('@01J+\r')
#        print('test22')
        imu_ang_z[i] = imu.grab_ang_r().z #this is the line failing 
#        print('test33')
        time.sleep(0.009)
    t1=time.time()
    ptu.cmd('@01STOP\r')
    hz = N/(t1-t0)
    data[str(run)]['elapsed']=np.linspace(0,t1-t0,len(imu_ang_z))
    data[str(run)]['imu_ang_z']=imu_ang_z
    cnt+=1
    time.sleep(2)
    print(cnt)

time.sleep(0.5)

plt.figure()
for run in test_vel:
    x=data[str(run)]['elapsed']
    y1=data[str(run)]['imu_ang_z']*180/np.pi
    y2=data[str(run)]['imu_ang_z'].rolling(1000,center=True).mean()*180./np.pi
    
    plt.plot(x,y1,'-o',label='imu_ang_z for ptu cmd= '+str(run)+' pos/sec')
    plt.plot(x,y2,'-o',label='imu_ang_z rolling for ptu cmd= '+str(run)+' pos/sec')
    data[str(run)].to_csv(cwd + '/testing/imu_z_step_response_ptu_vel_'+str(run)+'.csv')
plt.title('IMU data: sampling rate='+str(hz)+'hz\n sampling time='+str(round(t1-t0,4))+'seconds')
plt.legend()
#change baudrate back to default
#imu.change_baudrate(115200)   




