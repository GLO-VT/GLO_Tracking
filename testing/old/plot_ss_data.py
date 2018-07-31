# -*- coding: utf-8 -*-
"""
Created on Thu Jun 28 13:40:40 2018

@author: Bailey group PC
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from glob import glob
import os
import seaborn as sns
cwd = os.getcwd()
import matplotlib as mpl


################################################################################
base_size=30
mpl.rcParams['legend.fontsize'] = base_size
mpl.rcParams['figure.figsize'] = (25,15)
mpl.rcParams['figure.titlesize']=base_size+5
mpl.rcParams['xtick.labelsize']=base_size
mpl.rcParams['ytick.labelsize']=base_size
mpl.rcParams['font.size']=base_size
mpl.rcParams['axes.titlesize']=base_size
mpl.rcParams['axes.labelsize']=base_size
mpl.rcParams['lines.markersize'] = 4           # markersize, in points
mpl.rcParams['legend.markerscale'] = 1     # line width in points
mpl.rcParams['lines.markeredgewidth'] = 0.2 # the line width around the marker symbol
mpl.rcParams['lines.linewidth'] = 3.0
#####################################

#Function to organize tracking data
def grab_data(data_loc,header_len=2):
    '''
    Given a location of sun sensor .csv files, 
    '''

    #params = pd.read_csv(params_loc,skiprows=header_len)
    file_locs = glob(data_loc+'/*')
    runs=0
    if(len(file_locs)!=0): #only check list if is not empty, if empty leave run number as zero
        for i in range(len(file_locs)):
            run = file_locs[i].split('RUN')[-1].split('.')[0]
            if int(run) >= runs: 
                runs = int(run)
    

    data={}
    data_all=pd.DataFrame()
    data_all2=pd.DataFrame()
    runs_x=[]
    runs_y=[]
    ptu_x=[]
    ptu_y=[]
    imu_ang_z=[]
    
    for i in range(runs+1):
        run='run'+str(i)
        data[run]={}
        for file in glob(data_loc+'/*RUN'+str(i)+'*'):  
            for ss in ['ss1','ss2','ss3']:
                if ss in file:
                    data[run][ss]={}
                    with open(file) as myfile:
                        labels = [next(myfile) for x in range(1)][0].split(',')
                        values = [next(myfile) for x in range(1)][0].split(',')
#                    print(labels)
#                    print(values)
                    data[run][ss]['data']=pd.read_csv(file,index_col='time',header=header_len)
                    try:
                        #Convert imu z-axis angular rate from rad/sec to deg/sec
                        data[run][ss]['data']['imu_ang_z']=data[run][ss]['data']['imu_ang_z']*180.0/np.pi
                        #Smooth out angular rate and take gradient to calculate yaw accelerations in deg/sec^2
                        data[run][ss]['data']['accel']=np.gradient(data[run][ss]['data']['imu_ang_z'].rolling(30,center=True).mean(),data[run][ss]['data']['elapsed'])
                    except:
                        print('No IMU data for ',ss,file[i],i)
                        pass
                    data[run][ss]['data']['ss']=ss
                    data[run][ss]['data']['kpx']=float(values[0])
                    data[run][ss]['data']['kpy']=float(values[1])
                    data[run][ss]['data']['kix']=float(values[2])
                    data[run][ss]['data']['kiy']=float(values[3])
                    data[run][ss]['data']['kdx']=float(values[4])
                    data[run][ss]['data']['kdy']=float(values[5])
                    data[run][ss]['data']['hz']=int(values[6])
                    data[run][ss]['data']['run']=int(values[7])
                    data[run][ss]['data']['track_mode']=int(values[8])
                    data[run][ss]['data']['filter_mode']=int(values[9])
                    data[run][ss]['data']['track_time']=float(values[10])
                    print(values)
                    print(len(values))
                    print(float(values[13].split(' ')[-1].split(')')[0]))
                    data[run][ss]['data']['eshim_x_ss1']=float(values[11].split('(')[-1])
                    data[run][ss]['data']['eshim_x_ss2']=float(values[12].split(' ')[-1])
                    data[run][ss]['data']['eshim_x_ss3']=float(values[13].split(' ')[-1].split(')')[0])
                    data[run][ss]['data']['eshim_y_ss1']=float(values[14].split('(')[-1])
                    data[run][ss]['data']['eshim_y_ss2']=float(values[15].split(' ')[-1])
                    data[run][ss]['data']['eshim_y_ss3']=float(values[16].split(' ')[-1].split(')')[0])
                    if ss=='ss1':
                        runs_x.append(data[run][ss]['data']['ang_x_track'].values)
                        runs_y.append(data[run][ss]['data']['ang_y_track'].values)
                        ptu_x.append(data[run][ss]['data']['ang_x_track'].values)
                        ptu_y.append(data[run][ss]['data']['ang_x_track'].values)
                        imu_ang_z.append(data[run][ss]['data']['ang_x_track'].values)
                    data_all = data_all.append(data[run][ss]['data'])
                        #data_all['elapsed']=data_all
    

        
    data_all.index = pd.to_datetime(data_all.index)
    data_all['elapsed']=(data_all.index-data_all.index[0]).seconds+(data_all.index-data_all.index[0]).microseconds/1e6
    data_all['diff']=data_all['elapsed'].diff()
    data_all2 = pd.DataFrame()
#    data_all_ss1 = data_all.loc[data_all['ss']=='ss1',:]
#    data_all_ss2 = data_all.loc[data_all['ss']=='ss2',:]
#    data_all_ss3 = data_all.loc[data_all['ss']=='ss3',:]
    return data,data_all,runs_x,runs_y




header_len=2
#data_loc_rpi = cwd + '/rpi_tracking_20180619/20180522/'
#params_loc_rpi = cwd + '/20180619_Sun_Sensor_rpi_runs.txt'
#data_rpi,data_rpi_all_ss1,data_rpi_all_ss2,data_rpi_all_ss3,data_rpi_all = grab_data(data_loc_rpi,params_loc_rpi)
data_loc = cwd
#params_loc = cwd + '/20180619_Sun_Sensor_laptop_runs.txt'
data,data_all,runs_x,runs_y = grab_data(data_loc,header_len=header_len)
data_all_ss1 = data_all.loc[data_all['ss']=='ss1',:]
data_all_ss2 = data_all.loc[data_all['ss']=='ss2',:]
data_all_ss3 = data_all.loc[data_all['ss']=='ss3',:]

runs_all_x = pd.DataFrame()
runs_all_y = pd.DataFrame()
box_plot_all=True

box_plot_runs=[1,2,3]

try:
    if box_plot_all == True:
        N = range(len(runs_x))
    else:
        N = box_plot_runs
    for i in N:
        runs_all_x['run'+str(i)]=runs_x[i]
        runs_all_y['run'+str(i)]=runs_y[i]
        
    # make boxplot with Seaborn
    bplot=sns.boxplot(data=runs_all_x, 
                     width=0.5,
                     palette="colorblind")
except:
    print('could not create separte runs, maybe the runs lengths are different')

#print('Laptop Stats 95th percentile')
#plt.figure()
#bp=[] #list to store boxplots in
#for i in range(data_all_ss1['run'].max()):
#    mask = (data_all_ss1['run'] == i+1)
#    delay = int((1/data_all_ss1['hz'][0])*20)  #Allow for 20 second settling period
#    percentile=0.95
#    p_x=round(data_all_ss1.loc[mask,'ang_x_raw'][delay:].quantile(percentile),4)
#    p_y=round(data_all_ss1.loc[mask,'ang_y_raw'][delay:].quantile(percentile),4)
#    try:
#        print('run',i+1,
#              'x-off=',p_x,
#              'y-off=',p_y,
#              'kpx=',data_all_ss1.loc[mask,'kpx'][0],
#              'kpy=',data_all_ss1.loc[mask,'kpy'][0],
#              'kdx=',data_all_ss1.loc[mask,'kdx'][0],
#              'kdy=',data_all_ss1.loc[mask,'kdy'][0])
#        bp.append(data_all_ss1.loc[mask,'ang_x_raw'][delay:])
#    except:
#        pass
#plt.boxplot(bp) #plot boxplots of all data