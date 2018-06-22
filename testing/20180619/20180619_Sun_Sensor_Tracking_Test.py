# -*- coding: utf-8 -*-
"""
Created on Tue Jun 19 15:54:32 2018

@author: addiewan
"""

from glob import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
#from sklearn import linear_model
import os
import matplotlib as mpl
cwd = os.getcwd()

################################################################################
base_size=15
mpl.rcParams['legend.fontsize'] = base_size
mpl.rcParams['figure.figsize'] = (15,10)
mpl.rcParams['figure.titlesize']=base_size+5
mpl.rcParams['xtick.labelsize']=base_size
mpl.rcParams['ytick.labelsize']=base_size
mpl.rcParams['font.size']=base_size
mpl.rcParams['axes.titlesize']=base_size
mpl.rcParams['axes.labelsize']=base_size
mpl.rcParams['lines.markersize'] = 4           # markersize, in points
mpl.rcParams['legend.markerscale'] = 1     # line width in points
mpl.rcParams['lines.markeredgewidth'] = 0.2 # the line width around the marker symbol
mpl.rcParams['lines.linewidth'] = 1.5
#####################################

#Function to organize tracking data
def grab_data(data_loc,params_loc):
    '''
    Given a location of sun sensor .csv files, 
    '''
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

data_loc_rpi = cwd + '/rpi_tracking_20180619/20180522/'
params_loc_rpi = cwd + '/20180619_Sun_Sensor_rpi_runs.txt'
data_rpi,data_rpi_all_ss1,data_rpi_all_ss2,data_rpi_all_ss3,data_rpi_all = grab_data(data_loc_rpi,params_loc_rpi)
data_loc = cwd + '/laptop_tracking_20180619/'
params_loc = cwd + '/20180619_Sun_Sensor_laptop_runs.txt'
data,data_all_ss1,data_all_ss2,data_all_ss3,data_all = grab_data(data_loc,params_loc)

run_times = sort(list(data['ss1'].keys()))

#The tracking code does not appear to be running at desired sampling frequency
#Laptop tracking running ~7.6hz...
plt.figure()
start=3803
stop=4079
mask = (data_all_ss1['elapsed'] > start) & (data_all_ss1['elapsed'] < stop)
x=data_all_ss1.loc[mask,'elapsed']
y=data_all_ss1.loc[mask,'diff']
plt.plot(x,y,'o',label='Time Difference b/w succesive samples')
plt.title('Laptop Samppling Frequency')
plt.xlabel('Time (seconds)')
plt.ylabel('Seconds')
plt.ylim((0.09,0.17))
plt.legend()

#Tracking on raspberry pi running ~5hz
plt.figure()
start=305
stop=430
mask = (data_rpi_all_ss1['elapsed'] > start) & (data_rpi_all_ss1['elapsed'] < stop)
x=data_rpi_all_ss1.loc[mask,'elapsed']
y=data_rpi_all_ss1.loc[mask,'diff']
plt.plot(x,y,'o',label='Time Difference b/w succesive samples')
plt.title('Raspberry Pi Samppling Frequency')
plt.xlabel('Time (seconds)')
plt.ylabel('Seconds')
plt.ylim((0.150,0.275))
plt.legend()

#Tracking was run without electronic shims
#Plot the x-angle offset difference between sun sunsors
plt.figure()
start=3800
stop=4100
mask = (data_all_ss1['elapsed'] > start) & (data_all_ss1['elapsed'] < stop)
x=data_all_ss1.loc[mask,'elapsed']
y2_1=data_all_ss2.loc[mask,'ang_x_raw'] - data_all_ss1.loc[mask,'ang_x_raw']
y3_1=data_all_ss3.loc[mask,'ang_x_raw'] - data_all_ss1.loc[mask,'ang_x_raw']
plt.plot(x,y2_1,'o',label='ss2 - ss1 (x-axis)')
plt.plot(x,y3_1,'o',label='ss3 - ss1 (x-axis)')
plt.ylim((-0.1,0.1))
plt.legend()


plt.figure()
y2_1=data_all_ss2.loc[mask,'ang_y_raw'] - data_all_ss1.loc[mask,'ang_y_raw']
y3_1=data_all_ss3.loc[mask,'ang_y_raw'] - data_all_ss1.loc[mask,'ang_y_raw']
plt.plot(x,y2_1,'o',label='ss2 - ss1 (y-axis)')
plt.plot(x,y3_1,'o',label='ss3 - ss1 (y-axis)')
plt.ylim((-0.2,0.1))
plt.legend()

ss_eshim_x = [0.0,0.0,0.0]          #Specify electronic shims (x-dir) for sun sensors
ss_eshim_y = [0.0,0.0,0.0] 

#Plot raw x-angle offset for SS1 for all runs
#It is clear that the added weights helped the tracking significantly
plt.figure()
start=0
stop=data_all_ss1['elapsed'][-1]
mask_70 = data_all_ss1['run'] <= 8
mask_210 = data_all_ss1['run'] > 8
x_70=data_all_ss1.loc[mask_70,'elapsed']
x_210=data_all_ss1.loc[mask_210,'elapsed']
y_70=data_all_ss1.loc[mask_70,'ang_x_raw']
y_210=data_all_ss1.loc[mask_210,'ang_x_raw']
plt.plot(x_70,y_70,'o',label='ang_x_raw 70 lbs')
plt.plot(x_210,y_210,'o',label='ang_x_raw 210 lbs')
plt.xlabel('Time (seconds)')
plt.ylabel('Degrees')
for i in range(data_all_ss1['run'].max()):
    mask_run=data_all_ss1['run'] == i+1
    plt.axvline(data_all_ss1.loc[mask_run,'elapsed'][0],color='k')
plt.title('Raw x-angle offset SS1 for all runs')
plt.ylim((-1,1))
plt.legend()

plt.figure()
y_70=data_all_ss1.loc[mask_70,'ang_y_raw']
y_210=data_all_ss1.loc[mask_210,'ang_y_raw']
plt.plot(x_70,y_70,'o',label='ang_y_raw 70 lbs')
plt.plot(x_210,y_210,'o',label='ang_y_raw 210 lbs')
plt.xlabel('Time (seconds)')
plt.ylabel('Degrees')
for i in range(data_all_ss1['run'].max()):
    mask_run=data_all_ss1['run'] == i+1
    plt.axvline(data_all_ss1.loc[mask_run,'elapsed'][0],color='k')
plt.title('Raw x-angle offset SS1 for all runs')
plt.ylim((-1,1))
plt.legend()

plt.figure()
y_70=np.sqrt(data_all_ss1.loc[mask_70,'ang_y_raw']**2 + data_all_ss1.loc[mask_70,'ang_y_raw']**2)
y_210=np.sqrt(data_all_ss1.loc[mask_210,'ang_y_raw']**2 + data_all_ss1.loc[mask_210,'ang_y_raw']**2)
plt.plot(x_70,y_70,'o',label='ang_total_raw 70 lbs')
plt.plot(x_210,y_210,'o',label='ang_total_raw 210 lbs')
plt.xlabel('Time (seconds)')
plt.ylabel('Degrees')
for i in range(data_all_ss1['run'].max()):
    mask_run=data_all_ss1['run'] == i+1
    plt.axvline(data_all_ss1.loc[mask_run,'elapsed'][0],color='k')
plt.title('Raw total angle offset SS1 for all runs')
plt.ylim((0,1))
plt.legend()

print('Laptop Stats 95th percentile')
for i in range(data_all_ss1['run'].max()):
    mask = (data_all_ss1['run'] == i+1)
    delay = int(7.6*20)  #Allow for 20 second settling period
    percentile=0.95
    p_x=round(data_all_ss1.loc[mask,'ang_x_raw'][delay:].quantile(percentile),4)
    p_y=round(data_all_ss1.loc[mask,'ang_y_raw'][delay:].quantile(percentile),4)
    print('run',i+1,
          'x-off=',p_x,
          'y-off=',p_y,
          'kpx=',data_all_ss1.loc[mask,'kpx'][0],
          'kpy=',data_all_ss1.loc[mask,'kpy'][0],
          'kdx=',data_all_ss1.loc[mask,'kdx'][0],
          'kdy=',data_all_ss1.loc[mask,'kdy'][0])

print('')
print('Raspberry Pi Stats 95th percentile')    
for i in range(data_rpi_all_ss1['run'].max()):
    mask = (data_rpi_all_ss1['run'] == i+1)
    delay = int(7.6*20)  #Allow for 20 second settling period
    percentile=0.95
    p_x=round(data_rpi_all_ss1.loc[mask,'ang_x_raw'][delay:].quantile(percentile),4)
    p_y=round(data_rpi_all_ss1.loc[mask,'ang_y_raw'][delay:].quantile(percentile),4)
    print('run',i+1,
          'x-off=',p_x,
          'y-off=',p_y,
          'kpx=',data_rpi_all_ss1.loc[mask,'kpx'][0],
          'kpy=',data_rpi_all_ss1.loc[mask,'kpy'][0],
          'kdx=',data_rpi_all_ss1.loc[mask,'kdx'][0],
          'kdy=',data_rpi_all_ss1.loc[mask,'kdy'][0])

#Plot data for run 9: tracking within 0.1deg for 47% of time
plt.figure()
run=9
mask = data_all_ss1['run'] == run
x=data_all_ss1.loc[mask,'elapsed']
y=np.sqrt(data_all_ss1.loc[mask,'ang_x_raw']**2+data_all_ss1.loc[mask,'ang_y_raw']**2)
percentile=0.47
p=str(round(y.quantile(percentile),4))
plt.plot(x,y,'o',label='ang_y_raw SS1 run'+str(run)+'\n'+str(percentile*100)+'% percentile='+p+' deg')
plt.xlabel('Time (seconds)')
plt.ylabel('Degrees')
plt.title('Raw x-angle offset SS1 for run'+str(run))
plt.ylim((-0.0,0.4))
plt.legend()

#Plot data for run 9: accel vs. x-offset
plt.figure()
run=9
mask = data_all_ss1['run'] == run
x=data_all_ss1.loc[mask,'accel']
y=data_all_ss1.loc[mask,'ang_x_raw']
percentile=0.47
p=str(round(y.quantile(percentile),4))
plt.plot(x,y,'o')
plt.xlabel('Acceleration (deg/sec^2)')
plt.ylabel('Degrees')
plt.title('Run'+str(run)+' acceleration vs. x-offset')
plt.ylim((-0.4,0.4))

#for ss in ['ss1','ss2','ss3']:
#    for key in data[ss].keys():
#        ss_num = int(ss[-1]) - 1
#        x=data[ss][key]['elapsed']
#        y1=data[ss][key]['ang_x_raw']+ss_eshim_x[ss_num]
#        y2=data[ss][key]['ang_y_raw']+ss_eshim_y[ss_num]
#        try:
#            y3=data[ss][key]['accel']
#        except:
#            pass
#        
#        plt.figure(1)
#        plt.plot(x,y1,'o-',label='ss'+str(ss_num+1)+'_ang_x_raw')
#        try:
#            plt.plot(x,y3,'o-',label='ss'+str(ss_num+1)+'_accel')
#        except:
#            pass
#        plt.xlabel('Time Elapsed (seconds)')
#        plt.ylabel('Degrees')
#        plt.title('X-Axis sensor data at '+str(hz)+'hz\n kp='+str(kp_x)+' ki='+str(ki_x)+' kd='+str(kd_x))
#        plt.legend()
#        
#        plt.figure(2)
#        plt.plot(x,y2,'o-',label='ss'+str(ss_num+1)+'_ang_x_raw')
#        try:
#            plt.plot(x,y3,'o-',label='ss'+str(ss_num+1)+'_accel')
#        except:
#            pass
#        plt.xlabel('Time Elapsed (seconds)')
#        plt.ylabel('Degrees')
#        plt.title('Y-Axis sensor data at '+str(hz)+'hz\n kp='+str(kp_x)+' ki='+str(ki_x)+' kd='+str(kd_x))
#        plt.legend()
#    
##Plot acceleration vs. pan offset for all laptop runs
#plt.figure()
##Pick initial index to filter out 'settling period'
#start=400
#ss='ss2'
#for i in range(len(data[ss])):
#    try:
#        x=data[ss][i].accel[start:].values
#        y=data[ss][i].ang_x_raw[start:].values
#        x = x.reshape(len(x), 1)
#        y = y.reshape(len(y), 1)
#        regr = linear_model.LinearRegression()
#        regr.fit(x, y)
#        # plot it as in the example at http://scikit-learn.org/
#        plt.scatter(x, y,  color='black')
#        plt.plot(x, regr.predict(x), color='blue', linewidth=3,label=ss+' run '+str(i))
#        plt.xticks(())
#        plt.yticks(())
#        plt.show()
#    except:
#        pass
#    try:
#        plt.plot(x,y,'o',label='run'+str(i))
#    except:
#        pass
#    plt.legend()
#    
##Plot acceleration vs. pan offset for all laptop runs
#plt.figure()
##Pick initial index to filter out 'settling period'
#start=400
#ss='ss1'
#for i in range(len(data[ss])):
#    try:
#        x=data[ss][i].accel[start:]
#        y=data[ss][i].ang_x_raw[start:]
#        x = x[(~x.isnull() == True) & (~y.isnull() == True)]
#        y = y[(~x.isnull() == True) & (~y.isnull() == True)]
#        x = x.reshape(len(x), 1)
#        y = y.reshape(len(y), 1)
#        regr = linear_model.LinearRegression()
#        regr.fit(x, y)
#        plt.plot(x, regr.predict(x), linewidth=3,label=ss+' run '+str(i))
#    except:
#        pass
#    plt.legend()
#
##Plot acceleration vs. pan offset for all laptop runs
#plt.figure()
##Pick initial index to filter out 'settling period'
#start=400
#ss='ss1'
#for i in range(len(data[ss])):
#    try:
#        x=data[ss][i].accel[start:].values
#        y=data[ss][i].ang_x_raw[start:].values
#        plt.plot(x,y,'o',label='run'+str(i))
#    except:
#        pass
#    plt.legend()