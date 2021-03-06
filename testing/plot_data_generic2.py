# -*- coding: utf-8 -*-
"""
Created on Fri Aug 17 12:26:23 2018

@author: addiewan
"""
################################################################################
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl
import os
from glob import glob
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
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
################################################################################

#Plotting code

#Set location of .csv file (need r in front of string to handle windows slashes)
loc=r'G:\Team Drives\GLO IIP - Balloon Demonstration\12 - Fine Pointing System\C++ Tracking\Testing_8_17_18'
loc=r'G:\Team Drives\GLO IIP - Balloon Demonstration\12 - Fine Pointing System\C++ Tracking\Working Tracking'
files = glob(loc+'\*.csv')

data=[]
cnt=0
file = files[0]
df = pd.read_csv(file)
#for file in files:
##    if 'run'+str(run) in file.lower():
##        run_loc = file

#data.append(pd.read_csv(file))
#df = data[cnt]
mask=(df['cppFSS1_x'] != 0) & (np.abs(df['cppNewVelocity']) < 99999)

#x=df.loc[mask].index
x=df.loc[mask,'PID ellapseTime']
y1=df.loc[mask,'cppFSS1_x']
y2=df.loc[mask,'cppNewVelocity']
y3=df.loc[mask,'cppIMU1_AngularVelocity']

plt.figure()
#plt.plot(x,y3.rolling(10,center=True).mean(),'o',label='imu_ang_z rolling(10)')
plt.plot(x,y2,'o-',label='ptu_cmd_x')
plt.plot(x,y1,'o-',label='ss_off_x')
plt.xlabel('Time Elapsed (seconds)')
plt.ylabel('Degrees')
plt.hlines(0.1,x.values[0],x.values[-1],color='red')
plt.hlines(-0.1,x.values[0],x.values[-1],color='red')
plt.legend()
plt.grid()
plt.title(file)

plt.figure()
plt.hist(y1,bins=400)
plt.title('histogram of ss_x_off\n'+file)
cnt+=1