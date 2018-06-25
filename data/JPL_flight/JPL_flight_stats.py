# -*- coding: utf-8 -*-
"""
Created on Thu Jun 21 10:17:52 2018

@author: addiewan
"""

import numpy as np
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt
import pandas as pd
import os
cwd = os.getcwd()


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Filter requirements.
order = 6
fs = 2.6       # sample rate, Hz
cutoff = 0.1  # desired cutoff frequency of the filter, Hz

#Load JPL flight data
file_loc = cwd + "/eng16271.out"
data = pd.read_csv(file_loc,header=2,delim_whitespace=True)
mean_t_diff = data['tsecs'].diff().mean()  
hz = 1/mean_t_diff #data sampled at approximately 2.6hz
data = data.set_index('tsecs')   #set index to tsecs
data.index = data.index - data.index[0]  #start at t=0

#Pick a start and stop time (seconds) to get stats on acceleration
start = 8000
stop = 36128
percentile = 0.999  #set what percentile to calculate
#Filter data between start and stop times
data=data.loc[(data.index >= start) & (data.index <= stop),:]
 
n=len(data)   # total number of samples
t=data.index   #time in seconds

# Filter the data, and plot both the original and filtered signals.
butter_window=100
butter_window_accel=100
#Step 1: apply butterworth filter to absolute azimuth angle data (degrees)
data['butter'] = butter_lowpass_filter(data['AZIMUTH_Angle'], cutoff, fs, order)
#Step 2: Apply rolling mean to butter filtered data (butter filtered data is smoothed out, but still oscillates) 
data['rolled_butter']=data['butter'].rolling(window=butter_window,center=True).mean()
#Step 3: Take time derivative of azimuth angle to calculate angular rate (deg/sec)
data['az_ang_rate']=np.gradient(data['rolled_butter'],data.index)
#Step 4: Smooth out yaw angular rate (need to filter out noise before computing numerical derivative)
data['az_ang_rate']=data['az_ang_rate'].rolling(window=butter_window_accel,center=True).mean()
#Step 5: take time derivative of angular rate to get yaw angular acceleration
data['az_ang_accel'] = np.gradient(data['az_ang_rate'],data.index)

plt.figure()
x=data.index
plt.plot(x,data.AZIMUTH_Angle,label='yaw angle (raw) degrees')
plt.plot(x,data['butter'],label='yaw angle unsmoothed butterworth filter order='+str(order))
plt.plot(x,data['rolled_butter'],label='yaw angle (smoothed butter) degrees')
plt.plot(x,data['az_ang_rate'],label='angular rate of smoothed butter) deg/sec')
plt.plot(x,data['az_ang_accel'],label='angular accel of smoothed ang_rate) deg/sec^2')
plt.title('JPL Flight Data Acceleration estimation\n'+ str(percentile)+' percentile='+str(np.round(data['az_ang_accel'].quantile(percentile),4))+' deg/sec^2')
plt.xlabel('Time (seconds)')
plt.legend()

#Histogram of acceleration values
plt.figure()
plt.hist(data['az_ang_accel'],range=[0,0.002],bins=100)
plt.title('Histogram of acceleration values during "stable" portion of JPL flight\n'+ str(percentile)+' percentile='+str(np.round(data['az_ang_accel'].quantile(percentile),4))+' deg/sec^2')


plt.xlabel('Yaw acceleration (deg/sec^2)')