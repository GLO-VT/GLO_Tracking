# -*- coding: utf-8 -*-
"""
Created on Fri Jun 29 21:06:30 2018

@author: addiewan
"""
import pandas as pd
from glob import glob
import numpy as np

#Function to "organize" tracking data
def grab_data(data_loc,params_loc):
    params = pd.read_csv(params_loc)
    file_locs = glob(data_loc+'/*')
    data={}
    data_all=pd.DataFrame()
    data['ss1']={}
    data['ss2']={}
    data['ss3']={} 
    data['ss1_cnt']=1
    data['ss2_cnt']=1
    data['ss3_cnt']=1               
    for i in range(len(file_locs)):
        for ss in ['ss1','ss2','ss3']:
            if ss in file_locs[i]:
                temp = pd.read_csv(file_locs[i],index_col='time')
                key = temp.index[-1].split(' ')[1].split('.')[0]
                hour = int(key[0:2])
                minute = int(key[3:5])
                #Only include runs after 10:56
                if ((hour == 10) & (minute >= 56)) | (hour >= 11):
                    data[ss][key]=pd.read_csv(file_locs[i],index_col='time')
                    try:
                        #Convert imu z-axis angular rate from rad/sec to deg/sec
                        data[ss][key]['imu_ang_z']=data[ss][key]['imu_ang_z']*180.0/np.pi
                        #Smooth out angular rate and take gradient to calculate yaw accelerations in deg/sec^2
                        data[ss][key]['accel']=np.gradient(data[ss][key]['imu_ang_z'].rolling(30,center=True).mean(),data[ss][key]['elapsed'])
                        data[ss][key]['velocity']=np.gradient(data[ss][key]['ang_x_track'].rolling(30,center=True).mean(),data[ss][key]['elapsed'])
                        data[ss][key]['accel2']=np.gradient(data[ss][key]['velocity'].rolling(30,center=True).mean(),data[ss][key]['elapsed'])
                    except:
                        #print('No IMU data for ',ss,file_locs[i],i)
                        pass
                    data[ss][key]['ss']=ss
                    data[ss][key]['run']=data[ss+'_cnt']
                    mask = params['time_end']==key
                    data[ss][key]['freq']=params.loc[mask,'freq'].values[0]
                    data[ss][key]['kpx']=params.loc[mask,'kpx'].values[0]
                    data[ss][key]['kpy']=params.loc[mask,'kpy'].values[0]
                    data[ss][key]['kdx']=params.loc[mask,'kdx'].values[0]
                    data[ss][key]['kdy']=params.loc[mask,'kdy'].values[0]
                    data[ss][key]['imu']=params.loc[mask,'imu'].values[0]
                    data[ss][key]['weight']=params.loc[mask,'weight'].values[0]
                    data_all = data_all.append(data[ss][key])
                    data[ss+'_cnt']+=1
                    #data_all['elapsed']=data_all
    data_all.index = pd.to_datetime(data_all.index)
    data_all['elapsed']=(data_all.index-data_all.index[0]).seconds+(data_all.index-data_all.index[0]).microseconds/1e6
    data_all['diff']=data_all['elapsed'].diff()
    data_all_ss1 = data_all.loc[data_all['ss']=='ss1',:]
    data_all_ss2 = data_all.loc[data_all['ss']=='ss2',:]
    data_all_ss3 = data_all.loc[data_all['ss']=='ss3',:]
    return data,data_all_ss1,data_all_ss2,data_all_ss3,data_all

