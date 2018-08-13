# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 14:30:47 2018

@author: GLOtastic

"""
from imu import IMU
from pid import PID
from ptu_newmark import PTU
from ss import SS

import time
import random
import scipy.io as sio
import argparse
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import cv2
import ephem
import os
from glob import glob


class SS_tracking:
    '''
    PID tracking of the sun using a FLIR Pan Tilt Unit and feedback from sun sensors
    
    Inputs:
        ss: list of sun sensor serial connection objects 
        ptu: pan-tilt unit serial connection object
        ss_read: list of sun sensors to capture data from (ie ss_read=[1,2] will only read 
                 2 sun sensors with instrument ids of 1 and 2)
        ss_track: list of sun sensors to use for tracking (ie ss_track=[1,2] will only track 
                 with 2 sun sensors with instrument ids of 1 and 2)
        ss_eshim_x: list of x-axis "electronic shims (offset in degrees in sun 
                    sensor x-axis)" for all sun sensors corresponding to the
                    sun sensors listed in in ss_track
        ss_eshim_y: list of y-axis "electronic shims (offset in degrees in sun 
                    sensor y-axis)" for all sun sensors corresponding to the
                    sun sensors listed in in ss_track
        pid_x: PID class object for tracking in pan-axis
        pid_y: PID class object for tracking with tilt-axis
        ptu_cmd_delay: number of seconds to pause between a pan-axis and tilt-axis PTU command
        track mode (int):
           1: PID Position Control
           2: PID Absolute Velocity Control
           3: PID Velocity Derivative Control
           4: No tracking - Read Sun Sensor Data Only
           5: Ephemeris Tracking: Stationary platform
           6: Ephemeris Tracking: Moving platform (need GPS sensor)
        hz: sampling frequency (in hz)
        track_time: number of seconds to track and record data
        save_dir: directory to save tracking data to
        show_display: (boolean) set display on/off
        screen_res: screen resolution (default = (1280,800))
    '''
    def __init__(self,
                 ss,
                 ptu_x,
                 ptu_y,
                 imu,
                 ss_read=[1,2,3],
                 ss_track=[1,2,3],
                 ss_eshim_x=[0.0,0.0,0.0],
                 ss_eshim_y=[0.0,0.0,0.0],
                 filter_kern=[0.5,0.5],
                 pid_x=None,
                 pid_y=None,
                 ptu_cmd_delay=0.025,
                 track_mode=3,
                 hz=20,
                 track_time=120,
                 save_dir = 'C:/git_repos/GLO/Tutorials/tracking/',
                 track_x=True,
                 track_y=True,
                 screen_res=(1280,800)
                 ):
        
        #Initialize parameters
        self.ss = ss
        self.ptu_x = ptu_x
        self.ptu_y = ptu_y
        self.imu=imu
        self.ss_read = ss_read
        self.ss_track = ss_track
        self.ss_eshim_x = ss_eshim_x
        self.ss_eshim_y = ss_eshim_y
        self.filter_kern = filter_kern
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.ptu_cmd_delay=ptu_cmd_delay
        self.track_mode=track_mode
        self.hz=hz
        self.delay = 1.0/hz
        self.track_time=track_time
        self.save_dir = save_dir
        self.track_x = track_x
        self.track_y = track_y
        self.screen_res = screen_res
        
        #PtU initial paramaters
        try:
            self.ptu_x.cmd('@01STOP\r')   #Make sure PTU starts out stopped
        except:
            print('Could not Stop PTU pan axis, CHECK THE CABLES!!!')
        try:
            self.ptu_y.cmd('@01STOP\r')   #Make sure PTU starts out stopped
        except:
            print('Could not Stop PTU tilt axis, CHECK THE CABLES!!!')
        self.ptu_cmd_x = 0.0
        self.ptu_cmd_y = 0.0
        self.ptu_dir_x = 0
        self.ptu_dir_y = 0
        self.ptu_dir_x_new = 0
        self.ptu_dir_y_new = 0
        self.ptu_vel_lo = 15  #Set minimum velocity of PTU to 15 steps/sec
        self.ptu_vel_hi = 80000 #Set maximum velocity of ptu to 80000 steps/sec
        self.ptu_sat = False #gets set to True if PTU is saturated (0>ptu_vel<15 or ptu_vel>80000)
        self.pid_integrate = True  #Set to False to ignore integral PID gain 
        
        #Initialize PTU speed to 0
        self.spd_last_x = 0.0
        self.spd_last_y = 0.0
        
        self.t10=999
        self.t11=999
        
        #Initialized dataframe to store data  
        self.data = pd.DataFrame(columns=['ss_mean_x',
                                          'ss_mean_y',
                                          'ss_filt_x',
                                          'ss_filt_y',
                                          'ss1_x_raw',
                                          'ss1_y_raw',
                                          'ss2_x_raw',
                                          'ss2_y_raw',
                                          'ss3_x_raw',
                                          'ss3_y_raw',
                                          'ptu_cmd_x',
                                          'ptu_cmd_y',
                                          'ptu_pos_x',
                                          'ptu_pos_y',
                                          'ptu_dir_x',
                                          'ptu_dir_y',
                                          'imu_accel_x',
                                          'imu_accel_y',
                                          'imu_accel_z',
                                          'imu_ang_x',
                                          'imu_ang_y',
                                          'imu_ang_z',
                                          'imu_mag_x',
                                          'imu_mag_y',
                                          'imu_mag_z',
                                          'imu_ypr_x',
                                          'imu_ypr_y',
                                          'imu_ypr_z', 
                                          'imu_filt_x',
                                          'elapsed',
                                          't0',
                                          't1',
                                          't2',
                                          't3',
                                          't4',
                                          't5',
                                          't6',
                                          't7',
                                          't8',
                                          't9',
                                          't10',
                                          't11'
                                           ])
        
#    def setup_ptu(self):
#        '''
#        Set PTU to the appropriate control mode
#        Not currently used, need to change this logic from FLIR to Newmark logic
#        '''
#        try:
#            #Position mode
#            if self.track_mode == 1:
#                self.ptu_x.cmd('ci ')
#                time.sleep(0.1)
#                self.ptu_x.cmd('i ')
#                time.sleep(0.1)
#                self.ptu_x.cmd('ps1000 ')
#                time.sleep(0.1)
#                self.ptu_x.cmd('ts1000 ')
#            #Velocity Mode
#            if (self.track_mode == 2) | (self.track_mode == 3):
#                self.ptu_x.cmd('cv ')
#                time.sleep(0.1)
#                self.ptu_x.cmd('i ')
#                time.sleep(0.1)
#                self.ptu_x.cmd('ps0 ')
#                time.sleep(0.1)
#                self.ptu_x.cmd('ts0 ')
#        except:
#            sys.exit('Failed to set PTU control mode')
            
    def save_data(self):
        '''
        Check to see if directory for todays date exists, if not then create one
        and then save all data to a .csv file
        '''          
        file_time=time.strftime("%Y%m%d_%H%M%S")
        dir_date = time.strftime("%Y%m%d")+'/'
        if not os.path.exists(self.save_dir+dir_date):
            os.makedirs(self.save_dir+dir_date)  
        #find all csv files in today's folder 
        file_list = glob(self.save_dir+dir_date+ '/*.csv')
        run_number=0
        #loop through list 
        if(len(file_list)!=0): #only check list if is not empty, if empty leave run number as zero
            for i in range(len(file_list)):
                run = file_list[i].split('RUN')[-1].split('.')[0]
                if int(run) >= run_number: 
                    run_number = int(run)+1 #make the run number one larger than the largest
            
        #Save data to file   
        f_name=self.save_dir+dir_date+'ss_track_'+file_time+'_RUN'+str(run_number)+'.csv'
        print('saving tracking data to',f_name)
        self.data.to_csv(f_name,index_label='time')
        
        #Reopen data file and append header with various tracking run parameters
        df = pd.read_csv(f_name, header=None, index_col=None)
        n_cols=len(df.columns)
        print('df columns = ',len(df.columns))
        header=[]
        for j in range(n_cols):
            header.append('')
        #insert desired values into header[] in format set out below 
        header[0]=self.pid_x.Kp
        header[1]=self.pid_y.Kp
        header[2]=self.pid_x.Ki
        header[3]=self.pid_y.Ki
        header[4]=self.pid_x.Kd
        header[5]=self.pid_y.Kd
        header[6]=self.hz
        header[7]=run_number
        header[8]=self.track_mode
        header[9]=self.track_time
        header[10]=self.ss_eshim_x
        header[11]=self.ss_eshim_y
        df.columns = header
        df.to_csv(f_name, index=False)  #add line 1 of header
        
        df = pd.read_csv(f_name, header=None, index_col=None)
        n_cols=len(df.columns)
        header=[]
        for j in range(n_cols):
            header.append('')
        #add whatever strings to header 
        header[0]='kpx'
        header[1]='kpy'
        header[2]='kix'
        header[3]='kiy'
        header[4]='kdx'
        header[5]='kdy'
        header[6]='hz'
        header[7]='run'
        header[8]='track_mode'
        header[9]='track_time'
        header[10]='eshim_x'
        header[11]='eshim_y'
        df.columns = header
        df.to_csv(f_name, index=False) #add line 2 of header
        
        #Save data as matlab .mat file for simulation
        try:
            print('Saving .mat file to ',cwd+'/run'+str(run_number)+'.mat')
            sio.savemat(self.save_dir+dir_date+'ss_track_'+file_time+'_RUN'+str(run_number)+'.mat',
                        {'elapsed':self.data['elapsed'].values,
                         'imu_filt_x':self.data['imu_filt_x'].values,
                         'imu_ang_x':self.data['imu_ang_x'].values,
                         'imu_ang_y':self.data['imu_ang_y'].values,
                         'imu_ang_z':self.data['imu_ang_z'].values,
                         'ss_filt_x':self.data['ss_filt_x'].values,
                         'ss_filt_y':self.data['ss_filt_y'].values,
                         'ss_mean_x':self.data['ss_mean_x'].values,
                         'ss_mean_y':self.data['ss_mean_y'].values,
                         'ptu_cmd_x':self.data['ptu_cmd_x'].values,
                         'ptu_cmd_y':self.data['ptu_cmd_y'].values,
                         'ptu_pos_x':self.data['ptu_pos_x'].values,
                         'ptu_pos_y':self.data['ptu_pos_y'].values,
                         'ptu_dir_x':self.data['ptu_dir_x'].values,
                         'ptu_dir_y':self.data['ptu_dir_y'].values,
                         'kpx':self.pid_x.Kp,
                         'kpy':self.pid_y.Kp,
                         'kix':self.pid_x.Ki,
                         'kiy':self.pid_y.Ki,
                         'kdx':self.pid_x.Kd,
                         'kdy':self.pid_y.Kd,
                         'hz':self.hz,
                         'track_mode':self.track_mode,
                         'track_time':self.track_time,
                         'ss_eshim_x':self.ss_eshim_x,
                         'ss_eshim_y':self.ss_eshim_y})   
        except:
            print('Could not save .mat file')
    
    def ptu_stop(self,axis='x'):
        '''
        Stop PTU and set ptu_dir to 0
        Specify axis = 'x' or 'y' corresponding to 'pan' or 'tilt' axis
        '''
        if axis == 'x':
            ptu_response = self.ptu_x.cmd('@01STOP\r',delay=self.ptu_cmd_delay)
            if ptu_response == 'OK':
                self.ptu_dir_x_new=0
            else:
                self.ptu_dir_x_new = self.ptu_dir_x
        if axis == 'y':
            ptu_response = self.ptu_y.cmd('@01STOP\r',delay=self.ptu_cmd_delay)
            if ptu_response == 'OK':
                self.ptu_dir_y_new=0
            else:
                self.ptu_dir_y_new = self.ptu_dir_y
        
    def ptu_set_speed(self,ptu_speed,axis='x'):
        '''
        Send absolute value of desired speed command to ptu 
        Specify axis = 'x' or 'y' corresponding to 'pan' or 'tilt' axis
        '''
        if axis == 'x':
            self.t10 = time.time()
            command = '@01SSPD'+str(int(np.abs(ptu_speed)))+'\r'
            self.ptu_x.cmd(command)   #send x-axis ptu velocity command (absolute value of ptu_cmd
            #self.ptu_x.cmd('@01SSPD40000\r')
            self.t11 = time.time()
        if axis == 'y':
            self.ptu_y.cmd('@01SSPD'+str(np.abs(self.ptu_cmd_x))+'\r')   #send y-axis ptu velocity command (absolute value of ptu_cmd
            
    def ptu_jog_pos(self,axis='x'):
        '''
        Jog PTU in positive direction, set ptu_dir=1
        Specify axis = 'x' or 'y' corresponding to 'pan' or 'tilt' axis
        '''
        if axis == 'x':
            ptu_response = self.ptu_x.cmd('@01J+\r')  #jog ptu in positive direction
            if ptu_response == 'OK':
                print(self.cnt,'OK')
                self.ptu_dir_x_new=1
            else:
                self.ptu_dir_x_new = self.ptu_dir_x
        if axis == 'y':
            ptu_response = self.ptu_y.cmd('@01J+\r')  #jog ptu in positive direction
            if ptu_response == 'OK':
                self.ptu_dir_y_new=1
            else:
                self.ptu_dir_y_new = self.ptu_dir_y
        
    def ptu_jog_neg(self,axis='x'):
        '''
        Jog PTU in positive direction, set ptu_dir=-1
        Specify axis = 'x' or 'y' corresponding to 'pan' or 'tilt' axis
        '''
        if axis == 'x':
            ptu_response = self.ptu_x.cmd('@01J-\r')  #jog ptu in negative direction
            if ptu_response == 'OK':
                self.ptu_dir_x_new=-1
            else:
                self.ptu_dir_x_new = self.ptu_dir_x
        if axis == 'y':
            self.ptu_y.cmd('@01J-\r')  #jog ptu in negative direction
            if ptu_response == 'OK':
                self.ptu_dir_y_new=-1
            else:
                self.ptu_dir_y_new = self.ptu_dir_y
        
    def ptu_max_speed(self,axis='x'):
        '''
        set PTU to max allowable speed (80000)
        Specify axis = 'x' or 'y' corresponding to 'pan' or 'tilt' axis
        '''
        if axis == 'x':
            self.ptu_x.cmd('@01SSPD80000\r')
        if axis == 'y':
            self.ptu_y.cmd('@01SSPD80000\r')

#    def sine_wave(self,mode=1):
#        '''
#        mode 1: slow speed test
#        mode 2: fast speed test
#        '''
#        dt = time.time()-self.t_start
#        amp = 4.0
#        period = 3.0
#        offset = amp*np.sin(dt*2*np.pi/period)
#        return offset
        
    
    def run(self):
        '''
        Start tracking loop
        '''
        #Initialize PTU
        #self.setup_ptu()
        
        #Reference time for elapsed tracking time
        self.t_start = time.time()   
        self.cnt = 0
        
        while True:   #Main tracking loop that cycles at set sampling frequency
            #Time reference to ensure tracking operates at approximately set sampling frequency
            self.t0 = time.time()
            
######################### Read Fine Sun Sensors ###############################
            ang_x = np.zeros(3,dtype=float)   
            ang_y = np.zeros(3,dtype=float)
            ang_x.fill(np.nan)
            ang_y.fill(np.nan)

            #Collect Sun Sensor data
            for i in ss_read:    #Loop through all sun sensors in ss_read
                self.ss[i-1].read_data_all()    #Read all data from sun sensor using SS class, correct for python 0-indexing 
                if i in self.ss_track:   #Only include x and y SS offsets if included in ss_track
                    ang_x[i-1] = self.ss[i-1].ang_x_raw + self.ss_eshim_x[i-1]
                    ang_y[i-1] = self.ss[i-1].ang_y_raw + self.ss_eshim_y[i-1]
                  
                    #Uncomment the next three lines to test a sine wave 
#                    offset = self.sine_wave()
#                    ang_x[i-1] = self.ss[i-1].ang_x_raw + offset
#                    ang_y[i-1] = self.ss[i-1].ang_y_raw + offset

######################## Take Mean of Fine Sun Sensors ########################           
#            #Take arithmetic mean of all sun sensors listed in ss_track
#            self.ss_mean_x = np.nanmean(ang_x)
#            self.ss_mean_y = np.nanmean(ang_y)
            
            #Take geometric mean of all sun sensors listed in ss_track
            num_x = np.count_nonzero(~np.isnan(ang_x)) #count number of non-nan elements in ang_x
            num_y = np.count_nonzero(~np.isnan(ang_y)) #count number of non-nan elements in ang_y
            self.ss_mean_x = np.nanprod(ang_x)**(1./num_x) #geometric mean
            self.ss_mean_y = np.nanprod(ang_y)**(1./num_y) #geometric mean
            
            #Read current PTU position - need to implement, see genie_track_newmark.py
            
            #Read current IMU values (IMU filters values internally)
            try:
                self.imu_ang_r = self.imu.grab_ang_r()
                self.imu_filt_x = self.imu_ang_r.z
            except:
                print('Could not grab IMU data, cnt=',self.cnt)
                self.imu_filt_x = np.nan
            
######################### Filter SS data ###################################### 
            #Use filter kernel to filter ss_mean_x and ss_mean_y
            #initialize sun sensor filtered x/y by muliplying the current 
            #ss_mean_x and ss_mean_y by the last index in the filter kernel
            self.ss_filt_x = self.filter_kern[-1]*self.ss_mean_x
            self.ss_filt_y = self.filter_kern[-1]*self.ss_mean_y
            
            #Now cycle through the rest of filter weights in filter_kern 
            if len(self.filter_kern) > 1:  #if filter window size = 1, then do no filtering
                if self.cnt > len(self.filter_kern):  #wait until enough samples are available to filter  
                    #loop through the rest of the filter kernel and multiply the 
                    #weights by the appropriate indices of the past ss_mean_x and ss_mean_y values
                    for i in np.arange(-1,-len(self.filter_kern),-1):
                        self.ss_filt_x += self.filter_kern[i-1]*self.data['ss_mean_x'][i]
                        self.ss_filt_y += self.filter_kern[i-1]*self.data['ss_mean_x'][i]
                else:
                    #Just use raw ss data (mean values) until enough samples have been collected (# of elements of filter_kern)
                    self.ss_filt_x = self.ss_mean_x
                    self.ss_filt_y = self.ss_mean_y
        
            self.t1 = time.time()
######################## PID Controller #######################################
            #Use ss_filt_x and ss_filt_y as input to PID controller to generate PID output
            try:
                if (self.ss_filt_x > -5) & (self.ss_filt_x < 5):
                    self.pid_out_x = self.pid_x.GenOut(self.ss_filt_x)  #generate x-axis control output in "degrees"
                    if self.track_mode == 3:
                        self.ptu_cmd_x = self.spd_last_x + self.pid_out_x*self.pid_x.deg2pos  #convert to PTU positions
                        self.spd_last_x = self.ptu_cmd_x
                    if self.track_mode == 4:
                        if self.imu_filt_x != np.nan:
                            self.ptu_cmd_x = self.pid_out_x*self.pid_x.deg2pos - self.imu_filt_x  #convert to PTU positions
                        else:
                            self.ptu_cmd_x = self.pid_out_x*self.pid_x.deg2pos  #ignore IMU data if nan
                    
                else:
                    print('Sun sensor x-axis outside FOV, cnt=',self.cnt)
                    self.pid_out_x = np.nan
                    self.ptu_cmd_x = np.nan
                    
                if (self.ss_filt_y > -5) & (self.ss_filt_y < 5):
                    self.pid_out_y = self.pid_y.GenOut(self.ss_filt_y)  #generate y-axis control output in "degrees"
                    if self.track_mode == 3:
                        self.ptu_cmd_y = self.spd_last_y + self.pid_out_y*self.pid_y.deg2pos  #convert to PTU positions
                        self.spd_last_y = self.ptu_cmd_y
                    if self.track_mode == 4:
                        self.ptu_cmd_y = self.pid_out_y*self.pid_y.deg2pos  #ignore IMU data if nan
                else:
                    print('Sun sensor y-axis outside FOV, cnt=',self.cnt)
                    self.pid_out_y = np.nan
                    self.ptu_cmd_y = np.nan  
                                   
            except:
                print('PID output generation failed, cnt=',self.cnt)
                self.pid_out_x = np.nan
                self.ptu_cmd_x = np.nan
                self.pid_out_y = np.nan
                self.ptu_cmd_y = np.nan
            
            self.t2 = time.time()
##################### PTU Logic ###############################################
            #Implement 'switch direction' logic for newmark PTU x-axis
            if self.track_x:
                self.t6 = time.time()
                if (self.ptu_dir_x < 0):
                    	if (-self.ptu_vel_hi <  self.ptu_cmd_x <  -self.ptu_vel_lo):
                    		self.ptu_set_speed(self.ptu_cmd_x,axis='x')
                    	if self.ptu_cmd_x < -self.ptu_vel_hi:
                    		self.ptu_max_speed(axis='x')
                    	if (-self.ptu_vel_lo <  self.ptu_cmd_x <  self.ptu_vel_lo):
                    		self.ptu_stop(axis='x')
                    	if (self.ptu_cmd_x >= self.ptu_vel_lo):
                    		self.ptu_stop(axis='x')
                    		self.ptu_set_speed(self.ptu_cmd_x,axis='x')
                    		self.ptu_jog_pos(axis='x')
                
                self.t7 = time.time()
                if self.ptu_dir_x > 0:
                    	if self.ptu_cmd_x <= -self.ptu_vel_lo:
                    		self.ptu_stop(axis='x')
                    		self.ptu_set_speed(-self.ptu_cmd_x,axis='x')
                    		self.ptu_jog_neg(axis='x')
                    	if (-self.ptu_vel_lo <  self.ptu_cmd_x <  self.ptu_vel_lo):
                    		self.ptu_stop(axis='x')
                    	if (self.ptu_vel_lo <= self.ptu_cmd_x <= self.ptu_vel_hi):
                    		self.ptu_set_speed(self.ptu_cmd_x,axis='x')
                    	if self.ptu_cmd_x > self.ptu_vel_hi:
                    		self.ptu_max_speed(axis='x')
                self.t8 = time.time()   	
                if self.ptu_dir_x == 0:
                    	if (-self.ptu_vel_hi <= self.ptu_cmd_x <= -self.ptu_vel_lo):
                    		self.ptu_set_speed(-self.ptu_cmd_x,axis='x')
                    		self.ptu_jog_neg(axis='x')
                    	if (self.ptu_vel_hi >= self.ptu_cmd_x >= self.ptu_vel_lo):
                    		self.ptu_set_speed(self.ptu_cmd_x,axis='x')
                    		self.ptu_jog_pos(axis='x')
                    	if (self.ptu_cmd_x < -self.ptu_vel_hi):
                    		self.ptu_max_speed(axis='x')
                    		self.ptu_jog_neg(axis='x')
                    	if (self.ptu_cmd_x > self.ptu_vel_hi):
                    		self.ptu_max_speed(axis='x')
                    		self.ptu_jog_pos(axis='x')
                self.t9 = time.time()
                self.ptu_dir_x = self.ptu_dir_x_new
                        
            #Implement 'switch direction' logic for newmark PTU y-axis      
            if self.track_y:
                if self.ptu_dir_y < 0:
                    	if (-self.ptu_vel_hi <  self.ptu_cmd_y <  -self.ptu_vel_lo):
                    		self.ptu_set_speed(self.ptu_cmd_y,axis='y')
                    	if self.ptu_cmd_y < -self.ptu_vel_hi:
                    		self.ptu_max_speed(axis='y')
                    	if (-self.ptu_vel_lo <  self.ptu_cmd_y <  self.ptu_vel_lo):
                    		self.ptu_stop(axis='y')
                    	if self.ptu_cmd_y >= self.ptu_vel_lo:
                    		self.ptu_stop(axis='y')
                    		self.ptu_set_speed(self.ptu_cmd_y,axis='y')
                    		self.ptu_jog_pos(axis='y')
                
                if self.ptu_dir_y > 0:
                    	if self.ptu_cmd_y <= -self.ptu_vel_lo:
                    		self.ptu_stop(axis='y')
                    		self.ptu_set_speed(-self.ptu_cmd_y,axis='y')
                    		self.ptu_jog_neg(axis='y')
                    	if (-self.ptu_vel_lo <  self.ptu_cmd_y <  self.ptu_vel_lo):
                    		self.ptu_stop(axis='y')
                    	if (self.ptu_vel_lo <= self.ptu_cmd_y <= self.ptu_vel_hi):
                    		self.ptu_set_speed(self.ptu_cmd_y,axis='y')
                    	if self.ptu_cmd_y > self.ptu_vel_hi:
                    		self.ptu_max_speed(axis='y')
                    	
                if self.ptu_dir_y == 0:
                    	if (-self.ptu_vel_hi <= self.ptu_cmd_y <= -self.ptu_vel_lo):
                    		self.ptu_set_speed(-self.ptu_cmd_y,axis='y')
                    		self.ptu_jog_neg(axis='y')
                    	if (self.ptu_vel_hi >= self.ptu_cmd_y >= self.ptu_vel_lo):
                    		self.ptu_set_speed(self.ptu_cmd_y,axis='y')
                    		self.ptu_jog_pos(axis='y')
                    	if (self.ptu_cmd_y < -self.ptu_vel_hi):
                    		self.ptu_max_speed(axis='y')
                    		self.ptu_jog_neg(axis='y')
                    	if (self.ptu_cmd_y > self.ptu_vel_hi):
                    		self.ptu_max_speed(axis='y')
                    		self.ptu_jog_pos(axis='y')
                            
                self.ptu_dir_y = self.ptu_dir_y_new
                
            self.t3 = time.time()
#################### PID Anti-Windup Logic ####################################                           
            #Implement PID anti-windup logic - turn off integral gain if PTU is saturated
            #Determine if PTU is saturated
            if np.abs(self.ptu_cmd_x) > self.ptu_vel_hi:
            	self.ptu_sat = True
            if np.abs(self.ptu_cmd_x) < self.ptu_vel_lo:
            	self.ptu_sat = True
            	
            #If PTU is saturated, and offset is in same direction as PTU command, then disable PID integral gain    
            if self.ptu_sat:
            	if (self.ss_filt_x * self.ptu_cmd_x) > 0:
                    self.pid_integrate = False
            
            #Use PID integral gain if PTU is not saturated
            if ~self.ptu_sat:
                self.pid_integrate = True
                
            self.t4 = time.time()
############################# Store Data ######################################               
            #Record time elapsed from start of tracking loop to store in dataframe
            self.elapsed = time.time() - self.t_start
            self.d_time = datetime.now()
            if self.cnt > 1:
                self.dt = self.elapsed - self.data['elapsed'][self.cnt-1]
            
#            try:
            #Grab extra data from IMU
            self.imu_accel=self.imu.grab_accel()
            self.imu_ypr=self.imu.grab_ypr()
            self.imu_mag=self.imu.grab_mag()
            
            if self.cnt < 1:
                self.t5=1.0
            #Create a list of all data to nicely add a row of data to the dataframe
            data_add = [self.ss_mean_x,
                        self.ss_mean_y,
                        self.ss_filt_x,
                        self.ss_filt_y,
                        ang_x[0],
                        ang_y[0],
                        ang_x[1],
                        ang_y[1],
                        ang_x[2],
                        ang_y[2],
                        self.ptu_cmd_x,
                        self.ptu_cmd_y,
                        np.nan,  #self.ptu_pos_x  NEED to add
                        np.nan,  #self.ptu_pos_y NEED to add
                        self.ptu_dir_x,
                        self.ptu_dir_y,
                        self.imu_accel.x,
                        self.imu_accel.y,
                        self.imu_accel.z,
                        self.imu_ang_r.x,
                        self.imu_ang_r.y,
                        self.imu_ang_r.z,
                        self.imu_mag.x,
                        self.imu_mag.y,
                        self.imu_mag.z,
                        self.imu_ypr.x,
                        self.imu_ypr.y,
                        self.imu_ypr.z, 
                        self.imu_filt_x,
                        self.elapsed,
                        self.t0,
                        self.t1,
                        self.t2,
                        self.t3,
                        self.t4,
                        self.t5,
                        self.t6,
                        self.t7,
                        self.t8,
                        self.t9,
                        self.t10,
                        self.t11
                        ]
#            except:
#                #print('Could not grab IMU data accel, ypr, and mag, cnt=',self.cnt)
#                data_add = [np.nan,
#                            np.nan,
#                            ang_x[0],
#                            ang_y[0],
#                            ang_x[1],
#                            ang_y[1],
#                            ang_x[2],
#                            ang_y[2],
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan,
#                            np.nan, 
#                            self.imu_filt_x,
#                            self.elapsed,
#                            ]

                
            #add row of data to dataframe
            self.data.loc[self.d_time] = data_add
            
            #increment tracking loop counter
            self.cnt+=1
            
            #Maintain desired tracking loop frequency
            t_diff = time.time() - self.t0
            if self.delay - t_diff > 0:
                time.sleep(self.delay - t_diff)
                print('sleeping for ',self.delay - t_diff)
            self.t5 = time.time()
            #Check to see if specified tracking time has expired
            if (time.time() - self.t_start) > self.track_time:
                #Stop PTU from moving after tracking completes
                try:
                    self.ptu_stop(self,axis='x')
                    self.ptu_stop(self,axis='y')
                except:
                    print('Could not send PTU zero speed command, watch your toes!')
                print('Tracking complete, thanks for playing!')
                return

                
if __name__ == '__main__':
    cwd = os.getcwd()   
    parser = argparse.ArgumentParser(description='Sun sensor tracking code\n'+
                                     'I will add description later, I promise...')
    
###### Operational parameters ###########
    parser.add_argument('-tx','--track_x',
                        default=True,
                        type=bool,
                        help='Tracking in x-axis')

    parser.add_argument('-ty','--track_y',
                        default=False,
                        type=bool,
                        help='Tracking in y-axis')
    
    parser.add_argument('-tm','--track_mode',   # 3 = differential velocity mode (no IMU)
                        default=3,              # 4 = IMU compensation
                        type=int,
                        help='Tracking mode')
    
    parser.add_argument('-fk','--filter_kern',
                        default=[0.5,0.5],
                        type=list,
                        help='Filter mode')
    
    parser.add_argument('-fw','--filter_win',
                        default=5,
                        type=int,
                        help='Filter window size')    
    
    parser.add_argument('-pm','--ptu_offset_mode',
                        default=0,
                        type=int,
                        help='PTU offset mode')

    parser.add_argument('-d','--display',
                        default='False',
                        type=str,
                        help='show display')
    
    parser.add_argument('-t','--track_time',
                        default=60,
                        type=float,
                        help='Total time to track (seconds)')
    
    parser.add_argument('-hz','--hz',
                        default=20,
                        type=float,
                        help='Tracking frequency (hz)')
    
    parser.add_argument('-s','--save_dir',
                        default=cwd+'/testing/',
                        type=str,
                        help='Directory to save data to')
    
    parser.add_argument('-r','--read_dir',
                        default=cwd+'/Data.txt',
                        type=str,
                        help='File to read camera position from')
    
#    parser.add_argument('-h','--help',
#                        default=False,
#                        type=bool,
#                        help='Display help')

###### PID parameters ###############
    parser.add_argument('-kpx','--kpx',
                        default=1.0,
                        type=float,
                        help='Proportional gain x-axis')
    
    parser.add_argument('-kpy','--kpy',
                        default=-0.0,
                        type=float,
                        help='Proportional gain y-axis')
    
    parser.add_argument('-kdx','--kdx',
                        default=0.0,
                        type=float,
                        help='Derivative gain x-axis')
    
    parser.add_argument('-kdy','--kdy',
                        default=-0.0,
                        type=float,
                        help='Derivative gain y-axis')
    
    parser.add_argument('-kix','--kix',
                        default=0.0,
                        type=float,
                        help='Integral gain x-axis')
    
    parser.add_argument('-kiy','--kiy',
                        default=0.0,
                        type=float,
                        help='Integral gain y-axis')

    
######## Sun sensor parameters #################
    
    parser.add_argument('-ss1_r','--ss1_read',
                        default=False,
                        type=bool,
                        help='Read data from SS1 (True/False)')
    
    parser.add_argument('-ss2_r','--ss2_read',
                        default=True,
                        type=bool,
                        help='Read data from SS2 (True/False)')
    
    parser.add_argument('-ss3_r','--ss3_read',
                        default=False,
                        type=bool,
                        help='Read data from SS3 (True/False)')
    
    parser.add_argument('-ss1','--ss1_track',
                        default=False,
                        type=bool,
                        help='Track with SS1 (True/False)')
    
    parser.add_argument('-ss2','--ss2_track',
                        default=True,
                        type=bool,
                        help='Track with SS2 (True/False)')
    
    parser.add_argument('-ss3','--ss3_track',
                        default=False,
                        type=bool,
                        help='Track with SS3 (True/False)')

#    ss_eshim_x = [-1.763, -1.547, -1.578]          #Specify electronic shims (x-dir) for sun sensors
#    ss_eshim_y = [-2.290, -2.377, -2.215]          #Specify electronic shims (y-dir) for sun sensors   
    parser.add_argument('-ss1_ex','--ss1_eshim_x',
                        default=0.0,
                        type=float,
                        help='SS1 electronic shim x-axis')
    
    parser.add_argument('-ss2_ex','--ss2_eshim_x',
                        default=-2.0,
                        type=float,
                        help='SS2 electronic shim x-axis')
    
    parser.add_argument('-ss3_ex','--ss3_eshim_x',
                        default=0.0,
                        type=float,
                        help='SS3 electronic shim x-axis')
    
    parser.add_argument('-ss1_ey','--ss1_eshim_y',
                        default=0.0,
                        type=float,
                        help='SS1 electronic shim y-axis')
    
    parser.add_argument('-ss2_ey','--ss2_eshim_y',
                        default=0.0,
                        type=float,
                        help='SS2 electronic shim y-axis')
    
    parser.add_argument('-ss3_ey','--ss3_eshim_y',
                        default=0.0,
                        type=float,
                        help='SS3 electronic shim y-axis')
    
    parser.add_argument('-ss1_c','--ss1_com_port',
                        default='COM8',
                        type=str,
                        help='SS1 comm port')
    
    parser.add_argument('-ss2_c','--ss2_com_port',
                        default='COM4',
                        type=str,
                        help='SS2 com port')
    
    parser.add_argument('-ss3_c','--ss3_com_port',
                        default='COM8',
                        type=str,
                        help='SS3 com port')
    
    parser.add_argument('-ss1_b','--ss1_baud_rate',
                        default=115200,
                        type=int,
                        help='SS1 baud_rate')
    
    parser.add_argument('-ss2_b','--ss2_baud_rate',
                        default=115200,
                        type=int,
                        help='SS2 baud_rate')
    
    parser.add_argument('-ss3_b','--ss3_baud_rate',
                        default=115200,
                        type=int,
                        help='SS3 baud_rate')
    
    parser.add_argument('-ss1_i','--ss1_inst_id',
                        default=1,
                        type=int,
                        help='SS1 instrument id')
    
    parser.add_argument('-ss2_i','--ss2_inst_id',
                        default=2,
                        type=int,
                        help='SS2 instrument id')
    
    parser.add_argument('-ss3_i','--ss3_inst_id',
                        default=3,
                        type=int,
                        help='SS3 instrument id')
    
###### IMU parameters ###########
    parser.add_argument('-imu_c','--imu_com_port',
                        default='COM7',
                        type=str,
                        help='IMU comm port')    
    
    parser.add_argument('-imu_b','--imu_baud_rate',
                        default=115200,
                        type=int,
                        help='IMU baud_rate')
    
    
###### PTU parameters ###########
    parser.add_argument('-ptu_xc','--ptu_x_com_port',
                        default='COM9',
                        type=str,
                        help='IMU comm port')    
    
    parser.add_argument('-ptu_xb','--ptu_x_baud_rate',
                        default=9600,
                        type=int,
                        help='IMU baud_rate')
    
    parser.add_argument('-ptu_yc','--ptu_y_com_port',
                        default='COM10',
                        type=str,
                        help='IMU comm port')    
    
    parser.add_argument('-ptu_yb','--ptu_y_baud_rate',
                        default=9600,
                        type=int,
                        help='IMU baud_rate')
    
    parser.add_argument('-ptu_d','--ptu_cmd_delay',
                        default=0.00,
                        type=float,
                        help='PTU command delay')
    
    parser.add_argument('-ptu_s','--ptu_step_size',
                        default=500,
                        type=int,
                        help='PTU step size (default=500)')

#    parser.add_argument('-ptu_m','--ptu_set_micro',
#                        default=False,
#                        type=bool,
#                        help='set PTU to microstep (eighth)')
    
    parser.add_argument('-ptu_lat','--ptu_lat',
                        default='37.205144',
                        type=str,
                        help='PTU latitude')

    parser.add_argument('-ptu_lon','--ptu_lon',
                        default='-80.417560',
                        type=str,
                        help='PTU longitude')
    
    parser.add_argument('-ptu_alt','--ptu_alt',
                        default=634,
                        type=int,
                        help='PTU altitude')
    
    parser.add_argument('-ptu_utc','--ptu_utc_off',
                        default=4,
                        type=int,
                        help='PTU UTC offset')

    
    params=parser.parse_args() 
    
#    if params.help:
#        print('Tracking Mode (use -tm= ):\n'+
#               '1: PTU position-command mode: Simple PID control of ss offset\n'+
#               '2: PTU velocity-command mode: -imu velocity + PID control of ss position\n'+
#               '3: PTU velocity-command mode: -imu velocity - derivative of ss_offset + PID control of ss position\n'+
#               '4: No tracking - Read Sun Sensor Data Only\n'+
#               '5: Ephemeris Tracking: Stationary platform\n'+
#               '6: Ephemeris Tracking: Moving platform (need GPS sensor)')
#        
#        print('Sun Sensor/IMU Filtering Mode (use -fm= ):\n'+
#                '1: Raw data: Use mean of raw data from all tracking sun sensors\n'+
#                '2: Rolling Mean: Apply rolling mean to last n samples (n=filter window size)\n'+
#                '3: Butterworth: Apply butterworth filter to last n samples (n=filter window size)\n')
#        
#        print('Filter window size (use -fw=)\n'+
#              'ie, -fw=4 will use a filter window size of 4')
#        sys.exit()
  
    #Define Modes  
    track_mode = params.track_mode  #default: 4 = no tracking
    ptu_offset_mode = params.ptu_offset_mode  #default: 0 no PTU offset prior to tracking
    
    #Initiate PID control loop
    #pan-axis (x-axis) PID gains
    pid_x= PID(step_size=params.ptu_step_size) #'eighth'    #pid_x will control azimuth ptu motor (assuming orientation of ss is correct)
    pid_x.SetKp(params.kpx) #0.44
    pid_x.SetKi(params.kix) #0.05*0
    pid_x.SetKd(params.kdx) #0.3
    
    #tilt-axis (y-axis) PID gains
    pid_y= PID(step_size=params.ptu_step_size)     #pid_y will control azimuth ptu motor (assuming orientation of ss is correct)
    pid_y.SetKp(params.kpy) #-0.44
    pid_y.SetKi(params.kiy) #0.01*0
    pid_y.SetKd(params.kdy)   #-0.3
    
    print('Pan axis (x-axis) PID gains kpx=',params.kpx,'kix=',params.kix,'kdx=',params.kdx)
    print('Tilt axis (t-axis) PID gains kpy=',params.kpy,'kiy=',params.kiy,'kdy=',params.kdy)

    #not used currently
    #Obtain ephemeris data
#    ep = ephem.Observer()

    #Establish communication with sun sensor/s - store in a list
    ss=[SS(inst_id=params.ss1_inst_id,com_port=params.ss1_com_port,baudrate=params.ss1_baud_rate),
        SS(inst_id=params.ss2_inst_id,com_port=params.ss2_com_port,baudrate=params.ss2_baud_rate),
        SS(inst_id=params.ss3_inst_id,com_port=params.ss3_com_port,baudrate=params.ss3_baud_rate)]
    
    #List of sun sensors to read data from
    ss_read = []
    if params.ss1_read:
        ss_read.append(1)
    if params.ss2_read:
        ss_read.append(2)
    if params.ss3_read:
        ss_read.append(3)
    
    #List of sun sensors to use for tracking (also need to check if data is being read from sensor)
    ss_track = []
    if params.ss1_read:
        if params.ss1_track:
            ss_track.append(1)
    if params.ss2_read:
        if params.ss2_track:
            ss_track.append(2)
    if params.ss3_read:
        if params.ss3_track:
            ss_track.append(3)
    

    print('eshims_x',[params.ss1_eshim_x,
                      params.ss2_eshim_x,
                      params.ss3_eshim_x])
    print('eshims_y',[params.ss1_eshim_y,
                      params.ss2_eshim_y,
                      params.ss3_eshim_y])
    
    #Establish communication with IMU
    imu=IMU(com_port=params.imu_com_port,baudrate=params.imu_baud_rate)
    
    #Establish communication with PTU
    try:
        ptu_x = PTU(com_port=params.ptu_x_com_port,
                    baudrate=params.ptu_x_baud_rate,
                    cmd_delay=params.ptu_cmd_delay)
    except:
        print('COULD NOT TALK TO PTU pan axis!!!')
        ptu_x=None
    try:
        ptu_y = PTU(com_port=params.ptu_y_com_port,
                    baudrate=params.ptu_y_baud_rate,
                    cmd_delay=params.ptu_cmd_delay)
    except:
        print('COULD NOT TALK TO PTU tilt axis!!!')
        ptu_y=None

    #Set ptu=None if not using tracking to ensure PTU is not moved after initial offset
    if track_mode == 4:
        ptu_x.ptu.close()
        ptu_y.ptu.close()
        ptu_x=None
        ptu_y=None
        print('Not tracking, so disconnecting from the PTU for safe measure')

    #Initiate PTU tracking
    ss_tracking = SS_tracking(ss,
                              ptu_x,
                              ptu_y,
                              imu,
                              ss_read=ss_read,
                              ss_track=ss_track,
                              ss_eshim_x=[params.ss1_eshim_x,
                                          params.ss2_eshim_x,
                                          params.ss3_eshim_x], 
                              ss_eshim_y=[params.ss1_eshim_y,
                                          params.ss2_eshim_y,
                                          params.ss3_eshim_y],
                              filter_kern=params.filter_kern,
                              pid_x=pid_x,
                              pid_y=pid_y,
                              ptu_cmd_delay=params.ptu_cmd_delay,
                              track_mode=track_mode,
                              hz=params.hz,
                              track_time=params.track_time,
                              save_dir=params.save_dir,
                              track_x=params.track_x,
                              track_y=params.track_y,
                              )
        
    print('Tracking with sun sensors',ss_track,'for',params.track_time,'seconds')
    
    #Begin PTU tracking
    ss_tracking.run()
    
    #Save data
    ss_tracking.save_data()
    
    #Grab data in dataframe
    df = ss_tracking.data
    
    df['ss_time'] = df['t1']-df['t0']
    df['pid_time'] = df['t2']-df['t1']
    df['ptu_time'] = df['t3']-df['t2']
    df['aw_time'] = df['t4']-df['t3']
    df['data_time'] = df['t5']-df['t4']
    df['move_neg'] = df['t7']-df['t6']
    df['move_pos'] = df['t8']-df['t7']
    df['stopeed'] = df['t9']-df['t8']
    df['setspeed'] = df['t11']-df['t10']
        
    #Close IMU connection
    try:
        imu.imu.disconnect()
    except:
        print('Could not disconnect from IMU')
    
    #Close PTU connections
    try:
        ptu_x.cmd('@01STOP\r')
        ptu_x.ptu.close()
    except:
        print('Could not disconnect from PTU Pan Axis')
    try:
        ptu_y.cmd('@01STOP\r')
        ptu_y.ptu.close()
    except:
        print('Could not disconnect from PTU Tilt Axis')
    
    #Close sun sensor connections
    for i in range(len(ss)):
        try:
            ss[i].ss.serial.close() 
        except:
            print('could not close sun sensor',i)

    ##################################### Plot Data ###########################        
    try:
        #Plot y_angle raw vs. filtered 
        x=df['elapsed']
        y1=df['imu_ang_z']*180./np.pi
        y2=df['imu_filt_x']*180./np.pi
#        y3=df['ss2_x_raw']
        y4=df['ss_filt_x']
#        y5=df['ss2_x_raw']
        y6=df['ptu_cmd_x']
        
        plt.figure()
        plt.plot(x,y1,'o-',label='imu_ang_z')
        plt.xlabel('Time Elapsed (seconds)')
        plt.ylabel('Degrees')
        plt.title('X-Axis sensor data at '+str(params.hz)+'hz\n kp='+str(params.kpx)+' ki='+str(params.kix)+' kd='+str(params.kdx))
        plt.legend()
        
        #plt.figure(2)
       # plt.plot(x,y2,'o-',label='imu_filt_x')
        plt.plot(x,y4,'o-',label='ss2_ss_filt_x')
       # plt.plot(x,y4,'o-',label='filtered ss')
        plt.plot(x,y6,'o-',label='ptu_cmd_x')
        plt.xlabel('Time Elapsed (seconds)')
        plt.ylabel('Degrees')
        plt.ylim((-3,3))
        #plt.title('Y-Axis sensor data at '+str(hz)+'hz\n kp='+str(params.kpy)+' ki='+str(params.kiy)+' kd='+str(params.kdy))
        plt.legend()
        
        plt.figure()
        plt.title('X-Axis sensor data at '+str(params.hz)+'hz\n kp='+str(params.kpx)+' ki='+str(params.kix)+' kd='+str(params.kdx))
        plt.plot(x,y4,'o-',label='ss2_ang_x_raw')
        plt.plot(x,y6,'o-',label='ptu cmd x')
        plt.legend()
    except:
        print('Failed to plot data')


## Add to help display  
#    if manual_config == True:
#        ptu_micro = int(input('Set PTU to microstep mode?:\n'+
#                               '0: No\n'+
#                               '1: Yes\n'+
#                               '>>> '))
#
#            
#        else:
#            filter_mode = default_filter_mode
#        
#        ptu_offset_mode = int(input('Select PTU offset mode:\n'+
#                       '0: No Pointing Offset\n'+
#                       '1: Point PTU at Sun\n'+
#                       '2: Point PTU at Moon\n'+
#                       '>>> '))  