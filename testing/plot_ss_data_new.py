# -*- coding: utf-8 -*-
"""
Created on Thu Jun 28 13:40:40 2018

@author: Bailey group PC
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from glob import glob
import os
cwd = os.getcwd()

################################################################################
base_size=20
mpl.rcParams['legend.fontsize'] = base_size
mpl.rcParams['figure.figsize'] = (30,24)
mpl.rcParams['figure.titlesize']=base_size+5
mpl.rcParams['xtick.labelsize']=base_size
mpl.rcParams['ytick.labelsize']=base_size
mpl.rcParams['font.size']=base_size
mpl.rcParams['axes.titlesize']=base_size
mpl.rcParams['axes.labelsize']=base_size
mpl.rcParams['lines.markersize'] = 4           # markersize, in points
mpl.rcParams['legend.markerscale'] = 1     # line width in points
mpl.rcParams['lines.markeredgewidth'] = 0.4 # the line width around the marker symbol
mpl.rcParams['lines.linewidth'] = 4
#####################################

#Function to organize tracking data
def grab_data(data_loc,header_len=2):
    '''
    Given a location of sun sensor .csv files, 
    '''

    #params = pd.read_csv(params_loc,skiprows=header_len)
    file_locs = glob(data_loc+'/*')
    print(file_locs)
    runs=0
    if(len(file_locs)!=0): #only check list if is not empty, if empty leave run number as zero
        for i in range(len(file_locs)):
            run = file_locs[i].split('RUN')[-1].split('.')[0]
            print(run)
            try:
                if int(run) >= runs: 
                    runs = int(run)
            except:
                pass
    

    data={}
    data_all=pd.DataFrame()
    x_raw_ss1  =[]
    y_raw_ss1  =[]
    x_track_ss1=[]
    y_track_ss1=[]
    ptu_cmd_x  =[]
    ptu_cmd_y  =[]
    imu_ang_z  =[]
    x_raw_ss2  =[]
    y_raw_ss2  =[]
    x_track_ss2=[]
    y_track_ss2=[]
    for i in range(runs+1):
        run='run'+str(i)
        print('crunching',run)
        data[run]={}
        for file in glob(data_loc+'/*RUN'+str(i)+'*'):  
            data[run]={}
            with open(file) as myfile:
                #Parse first row of ss data file for parameter labels
                labels = [next(myfile) for x in range(1)][0].split(',')
                #Parse second row of ss data file for parameter values
                values = [next(myfile) for x in range(1)][0].split(',')
                    data[run]['data']=pd.read_csv(file,index_col='time',header=header_len)
                    try:
                        #Convert imu z-axis angular rate from rad/sec to deg/sec
                        data[run]['data']['imu_ang_z']=data[run]['data']['imu_ang_z']*180.0/np.pi
                        #Smooth out angular rate and take gradient to calculate yaw accelerations in deg/sec^2
                        data[run]['data']['accel']=np.gradient(data[run]['data']['imu_ang_z'].rolling(30,center=True).mean(),data[run][ss]['data']['elapsed'])
                    except:
                        print('No IMU data for ',ss,file[i],i)
                        pass
                    data[run][ss]['data'].index = pd.to_datetime(data[run][ss]['data'].index)
                    data[run][ss]['data']['elapsed']=(data[run][ss]['data'].index-data[run][ss]['data'].index[0]).seconds+(data[run][ss]['data'].index-data[run][ss]['data'].index[0]).microseconds/1e6
                    #Load in parameters from header
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
                    data[run][ss]['data']['eshim_x_ss1']=float(values[11].split('(')[-1])
                    data[run][ss]['data']['eshim_x_ss2']=float(values[12].split(' ')[-1])
                    data[run][ss]['data']['eshim_x_ss3']=float(values[13].split(' ')[-1].split(')')[0])
                    data[run][ss]['data']['eshim_y_ss1']=float(values[14].split('(')[-1])
                    data[run][ss]['data']['eshim_y_ss2']=float(values[15].split(' ')[-1])
                    data[run][ss]['data']['eshim_y_ss3']=float(values[16].split(' ')[-1].split(')')[0])
                    if ss=='ss1':
                        x_raw_ss1.append(data[run][ss]['data']['ang_x_raw'].values)
                        y_raw_ss1.append(data[run][ss]['data']['ang_y_raw'].values)
                        x_track_ss1.append(data[run][ss]['data']['ang_x_track'].values)
                        y_track_ss1.append(data[run][ss]['data']['ang_y_track'].values)
                        ptu_cmd_x.append(data[run][ss]['data']['ptu_cmd_x'].values)
                        ptu_cmd_y.append(data[run][ss]['data']['ptu_cmd_y'].values)
                        imu_ang_z.append(data[run][ss]['data']['imu_ang_z'].values)
                    if ss=='ss2':
                        x_raw_ss2.append(data[run][ss]['data']['ang_x_raw'].values)
                        y_raw_ss2.append(data[run][ss]['data']['ang_y_raw'].values)
                        x_track_ss2.append(data[run][ss]['data']['ang_x_track'].values)
                        y_track_ss2.append(data[run][ss]['data']['ang_y_track'].values)
                     
                        
                    data_all = data_all.append(data[run][ss]['data'])
                    #data_all['elapsed']=data_all
    data_all.index = pd.to_datetime(data_all.index)
    data_all['elapsed']=(data_all.index-data_all.index[0]).seconds+(data_all.index-data_all.index[0]).microseconds/1e6
    data_all['diff']=data_all['elapsed'].diff()
#    data_all_ss1 = data_all.loc[data_all['ss']=='ss1',:]
#    data_all_ss2 = data_all.loc[data_all['ss']=='ss2',:]
#    data_all_ss3 = data_all.loc[data_all['ss']=='ss3',:]
    return {
            'data':       data,
            'data_all':   data_all,
            'x_raw_ss1':  x_raw_ss1,
            'y_raw_ss1':  y_raw_ss1,
            'x_track_ss1':x_track_ss1,
            'y_track_ss1':y_track_ss1,
            'ptu_cmd_x':  ptu_cmd_x,
            'ptu_cmd_y':  ptu_cmd_y,
            'imu_ang_z':  imu_ang_z,
            'x_raw_ss2':  x_raw_ss2,
            'y_raw_ss2':  y_raw_ss2,
            'x_track_ss2':x_track_ss2,
            'y_track_ss2':y_track_ss2,
            'num_runs':runs
            }





header_len=2
#data_loc_rpi = cwd + '/rpi_tracking_20180619/20180522/'
#params_loc_rpi = cwd + '/20180619_Sun_Sensor_rpi_runs.txt'
#data_rpi,data_rpi_all_ss1,data_rpi_all_ss2,data_rpi_all_ss3,data_rpi_all = grab_data(data_loc_rpi,params_loc_rpi)
data_loc =  cwd+'/20180709'
#params_loc = cwd + '/20180619_Sun_Sensor_laptop_runs.txt'
data = grab_data(data_loc,header_len=header_len)
#data_all_ss1 = data['data_all'].loc[data_all['ss']=='ss1',:]
#data_all_ss2 = data['data_all'].loc[data_all['ss']=='ss2',:]
#data_all_ss3 = data['data_all'].loc[data_all['ss']=='ss3',:]

#set which plots to display
plot_tracking_all=False
plot_hist_elapsed_diff=False
plot_accel_all=False
plot_accel_histogram_all=False

if plot_tracking_all:

    for i in range(data['num_runs']):
        plt.figure()
        kpx=str(data['data']['run'+str(i)]['ss1']['data']['kpx'][0])
        kpy=str(data['data']['run'+str(i)]['ss1']['data']['kpy'][0])
        kix=str(data['data']['run'+str(i)]['ss1']['data']['kix'][0])
        kiy=str(data['data']['run'+str(i)]['ss1']['data']['kiy'][0])
        kdx=str(data['data']['run'+str(i)]['ss1']['data']['kdx'][0])
        kdy=str(data['data']['run'+str(i)]['ss1']['data']['kdy'][0])
        plt.suptitle('Hang Test 2018-06-29 Run '+str(i)+'\nKpx='+kpx+
                                                            ' Kix='+kix+
                                                            ' Kdx='+kdx+
                                                            '\nKpy='+kpy+
                                                            ' Kiy='+kiy+
                                                            ' Kdy='+kdy)
        plt.subplot(3,1,1)
        x=data['data']['run'+str(i)]['ss1']['data']['elapsed']
        y1=data['data']['run'+str(i)]['ss1']['data']['imu_ang_z']*180./np.pi
        y2=(data['data']['run'+str(i)]['ss1']['data']['imu_ang_z']*180./np.pi).rolling(30,center=True).mean()
        y3=np.gradient(y2)
        plt.plot(x,y1,label='imu_ang_z')
        plt.plot(x,y2,label='imu_ang_z_rolling')
        plt.plot(x,y3,'-o',label='accel from rolling mean')
        plt.legend()
        
        plt.subplot(3,1,2)
        x=data['data']['run'+str(i)]['ss1']['data']['elapsed']
        y1=data['data']['run'+str(i)]['ss1']['data']['ang_x_raw']
        y2=data['data']['run'+str(i)]['ss1']['data']['ang_x_track']
        y3=data['data']['run'+str(i)]['ss2']['data']['ang_x_raw']
        #y4=data['data']['run'+str(i)]['ss2']['data']['ang_x_track']
        base_size=15
        plt.plot(x,y1,'o',markersize=base_size,label='ss1: ang_x_raw')
        plt.plot(x,y3,'o',markersize=base_size*0.8,label='ss2: ang_x_raw')
        plt.plot(x,y2,'-o',markersize=base_size*0.6,label='ang_x_track')
        plt.legend()
        
        plt.subplot(3,1,3)
        x=data['data']['run'+str(i)]['ss1']['data']['elapsed']
        y1=data['data']['run'+str(i)]['ss1']['data']['ang_y_raw']
        y2=data['data']['run'+str(i)]['ss1']['data']['ang_y_track']
        y3=data['data']['run'+str(i)]['ss2']['data']['ang_y_raw']
        #y4=data['data']['run'+str(i)]['ss2']['data']['ang_y_track']
        base_size=15
        plt.plot(x,y1,'o',markersize=base_size,label='ss1: ang_y_raw')
        plt.plot(x,y3,'o',markersize=base_size*0.8,label='ss2: ang_y_raw')
        plt.plot(x,y2,'-o',markersize=base_size*0.6,label='ang_y_track')
        plt.legend()

if plot_hist_elapsed_diff:
    for i in range(data['num_runs']):    
        plt.figure()
        x=data['data']['run'+str(i)]['ss1']['data']['elapsed'].diff()
        plt.hist(x,bins=100,range=(0.02,0.08))
        plt.title('Histogram of difference between succesive timestamps')

if plot_accel_all:
    for i in range(data['num_runs']):    
        plt.figure()
        plt.subplot(2,1,1)
        x=data['data']['run'+str(i)]['ss1']['data']['elapsed']
        y1=data['data']['run'+str(i)]['ss1']['data']['imu_ang_z']*180./np.pi
        y2=(data['data']['run'+str(i)]['ss1']['data']['imu_ang_z']*180./np.pi).rolling(30,center=True).mean()
        y3=np.gradient(y2)
        plt.plot(x,y1,label='imu_ang_z')
        plt.plot(x,y2,label='imu_ang_z_rolling')
        plt.plot(x,y3,'-o',label='accel from rolling mean')
        plt.legend()
        
        
        plt.subplot(2,1,2)
        x=np.gradient((data['data']['run'+str(i)]['ss1']['data']['imu_ang_z']*180./np.pi).rolling(30,center=True).mean())
        plt.hist(np.abs(x),bins=300,range=(0,2))
        plt.title('Run '+str(i)+'Histogram of accelerations (absolute value')

if plot_accel_histogram_all:
    plt.figure()
    x=data['data_all']['accel']
    plt.hist(x,bins=1000,range=(0,2))
    plt.title('Histogram of accelerations of all runs \n(acceleration calculated from rolling mean (window=30) of imu angular rate z-axis)')


x=np.array([0,1,2])
y=np.array([2,5,1])
x=np.linspace(0,2*np.pi,1000)
y=np.sin(x)

def ddx(x,y):
    return (np.array([2/((x[1]-x[0])*(x[2]-x[0])),
                    -2/((x[2]-x[1])*(x[1]-x[0])),
                    2/((x[2]-x[1])*(x[2]-x[0]))])*y.T).sum()
    
plt.figure()
#plt.plot(x*180./np.pi,y,label='y')
plt.plot(x*180./np.pi,np.gradient(np.gradient(y,x)),label='np.gradient(np.gradient)')
#plt.ylim((-2,2))

y_grad2=[]
y_grad2.append(0.0)
y_grad2.append(0.0)
for i in range(2,len(x)):
    y_grad2.append(np.gradient(np.gradient(y[i-2:i+1],x[i-2:i+1])).sum())
plt.plot(x*180./np.pi,y_grad2) 

y_numer2=[]
y_numer2.append(0.0)
y_numer2.append(0.0)  
for i in range(2,len(x)):
    y_numer2.append(ddx(x[i-2:i+1],y[i-2:i+1]))
plt.plot(x*180./np.pi,y_numer2)


#runs_all = pd.DataFrame()
#
#box_plot_all=True
#box_plot_runs=[1,2,2]
#
#
#try:
#    if box_plot_runs == True:
#        N= range(len(x_raw_ss1))
#    else:
#        N=bpx_plot_runs
#    for i in N:
#        runs_all['run'+str(i)]=x_raw_ss1
#except:
#    pass        
##    bplot
    


#print('Laptop Stats 95th percentile')
#plt.figure()
#bp=[] #list to store boxplots in
#for i in range(data_all_ss1['run'].max()):
#    mask = (data_all_ss1['run'] == i+1)
#    delay = int(data_all_ss1['hz'][0]*20)  #Allow for 20 second settling period
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