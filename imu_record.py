# -*- coding: utf-8 -*-
"""
Created on Thu May 31 10:57:21 2018

@author: Bailey group PC
"""
from imu import IMU
import time
import pandas as pd
from datetime import datetime
import os

#Create serial connection to IMU
imu = IMU(com_port='COM7',baudrate=115200) #921600

#Set recording parameters
hz=20
delay = 1.0/hz
record_time=5
save_dir = 'C:/git_repos/GLO/Tutorials/tracking/'

#Initiate pandas dataframe to store IMU data in
data = pd.DataFrame(columns=['accel_x',
                             'accel_y',
                             'accel_z',
                             'angr_x',
                             'angr_y',
                             'angr_z',
                             'mag_x',
                             'mag_y',
                             'mag_z',
                             'elapsed'])

t_start=time.time()
print('Recording VN100 IMU data for',record_time,'seconds')
while (time.time() - t_start) < record_time:
    
    #Grab time at beginning of each iteration to keep sampling time frequency consistent
    t0=time.time()
    
    #Grab imu data
    imu.grab_accel()
    imu.grab_ang_r()
    imu.grab_mag()
    
    #Grab timestamp and add row to dataframe
    elapsed = time.time() - t_start 
    data_add = [imu.accel.x,
                imu.accel.y,
                imu.accel.z,
                imu.ang_r.x,
                imu.ang_r.y,
                imu.ang_r.z,
                imu.mag.x,
                imu.mag.y,
                imu.mag.z,
                elapsed]
    data.loc[datetime.now()] = data_add
    
    #Sleep until sampling time has expired
    t_diff = time.time() - t0
    if delay - t_diff > 0:
        time.sleep(delay - t_diff)


#Check to see if directory for todays date exists, if not then create one
file_time=time.strftime("%Y%m%d_%H%M%S")
dir_date = time.strftime("%Y%m%d")+'/'
if not os.path.exists(save_dir+dir_date):
    os.makedirs(save_dir+dir_date)  

#Save data to file    
data.to_csv(save_dir+dir_date+'imu_'+file_time+'.csv',index_label='time')
    
#Disconnect from imu
imu.imu.disconnect()

