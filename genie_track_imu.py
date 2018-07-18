# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 14:30:47 2018

@author: GLOtastic

"""
from imu import IMU
from pid import PID
from ptu import PTU
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
        filter mode:
            1: Raw data: Use mean of raw data from all tracking sun sensors
            2: Filtered data: Use mean of filtered data from all tracking sun sensors
            3: Kalman Filter: probably not implemented yet...
        hz: sampling frequency (in hz)
        track_time: number of seconds to track and record data
        save_dir: directory to save tracking data to
        show_display: (boolean) set display on/off
        screen_res: screen resolution (default = (1280,800))
    '''
    def __init__(self,
                 ss,
                 ptu,
                 imu,
                 ss_read=[1,2,3],
                 ss_track=[1,2,3],
                 ss_eshim_x=[0.0,0.0,0.0],
                 ss_eshim_y=[0.0,0.0,0.0],
                 pid_x=None,
                 pid_y=None,
                 ptu_cmd_delay=0.025,
                 track_mode=3,
                 filter_mode=1,
                 filter_win=5,
                 hz=20,
                 track_time=120,
                 save_dir = 'C:/git_repos/GLO/Tutorials/tracking/',
                 show_display=True,
                 track_x=True,
                 track_y=True,
                 screen_res=(1280,800),
                 read_dir=None
                 ):
        
        #Initialize parameters
        self.ss = ss
        self.ptu = ptu
        self.imu=imu
        self.ss_read = ss_read
        self.ss_track = ss_track
        self.ss_eshim_x = ss_eshim_x
        self.ss_eshim_y = ss_eshim_y
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.ptu_cmd_delay=ptu_cmd_delay
        self.track_mode=track_mode
        self.filter_mode=filter_mode
        self.filter_win=filter_win
        self.hz=hz
        self.delay = 1.0/hz
        self.track_time=track_time
        self.save_dir = save_dir
        self.show_display = show_display
        self.track_x = track_x
        self.track_y = track_y
        self.max_vel_x = 1.0
        self.max_vel_y = 1.0
        self.screen_res = screen_res
        self.read_dir = read_dir
    
        #Initialized dataframe to store data  
        self.data = pd.DataFrame(columns=['ang_x_track',
                                          'ang_y_track',
#                                          'ss1_x_raw',
#                                          'ss1_y_raw',
#                                          'ss2_x_raw',
#                                          'ss2_y_raw',
#                                          'ss3_x_raw',
#                                          'ss3_y_raw',
                                          'ptu_cmd_x',
                                          'ptu_cmd_y',
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
                                          'imu_filt_y',
                                          'elapsed'])
                    
        #Initialize PTU speed to 0
        self.spd_last_x = 0.0
        self.spd_last_y = 0.0
        
        #Set sun sensor modbus registers to either collect raw data or fitered data
        if self.filter_mode == 1:
            self.reg_x = 5
            self.reg_y = 6
        if self.filter_mode == 2:
            self.reg_x = 3
            self.reg_y = 4

    def pid_pos(self,ang_x_track,ang_y_track):
        '''
        Track using position control
            Converts SS PID error signal into a PTU position offset command 
        '''
        #If SS angle offsets are within bounds, generate PID error signal and ptu command
        try:
            if (ang_x_track > -5) & (ang_x_track < 5):
                self.pid_out_x = self.pid_x.GenOut(ang_x_track)  #generate x-axis control output in degrees
            else:
                self.pid_out_x  = np.nan
          
            if (ang_y_track > -5) & (ang_y_track < 5):
                self.pid_out_y  = self.pid_y.GenOut(ang_y_track)  #generate y-axis control output in degrees
            else:
                self.pid_out_y = np.nan

        except:
            print('PID position output failed')
            self.pid_out_x = np.nan 
            self.pid_out_y = np.nan 
        
    def setup_display(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale=1
        self.img = np.zeros((int(self.screen_res[0]/2),int(self.screen_res[1]/2)),dtype='int8')
        cv2.namedWindow('doin stuff',cv2.WINDOW_NORMAL)
        
    def setup_ptu(self):
        '''
        Set PTU to the appropriate control mode
        '''
        try:
            #Position mode
            if self.track_mode == 1:
                self.ptu.cmd('ci ')
                time.sleep(0.1)
                self.ptu.cmd('i ')
                time.sleep(0.1)
                self.ptu.cmd('ps1000 ')
                time.sleep(0.1)
                self.ptu.cmd('ts1000 ')
            #Velocity Mode
            if (self.track_mode == 2) | (self.track_mode == 3):
                self.ptu.cmd('cv ')
                time.sleep(0.1)
                self.ptu.cmd('i ')
                time.sleep(0.1)
                self.ptu.cmd('ps0 ')
                time.sleep(0.1)
                self.ptu.cmd('ts0 ')
        except:
            sys.exit('Failed to set PTU control mode')

    def update_display(self):
        '''
        Update tracking viewer data
        '''
        ang_x = self.ang_x_track
        ang_y = self.ang_y_track
        ang_off = np.sqrt(ang_x**2 + ang_y**2)
        img=np.zeros((1000,1000),dtype='int8')
        if np.isfinite(ang_x) & np.isfinite(ang_y):
            pix_off_x = int((img.shape[0]/2) - (ang_x*50))
            pix_off_y = int((img.shape[1]/2) - (ang_y*50))
            pix_off_tot = int(ang_off*50)
            cv2.putText(img,'Offset X = '+str(round(ang_x,4))+ ' degrees',(int(img.shape[0]/2)-40,40), self.font, self.font_scale,(255,0,255),2,cv2.LINE_AA)
            cv2.putText(img,'Offset Y = '+str(round(ang_y,4))+ ' degrees',(int(img.shape[0]/2)-40,80), self.font, self.font_scale,(255,0,255),2,cv2.LINE_AA)
            cv2.putText(img,'Offset Total = '+str(round(ang_off,4))+ ' degrees',(int(img.shape[0]/2)-40,120), self.font, self.font_scale,(255,0,255),2,cv2.LINE_AA)
            cv2.putText(img,'PTU pan Speed = '+str(round(self.spd_last_x/self.pid_x.deg2pos,3))+ ' deg/sec',(int(img.shape[0]/2)-40,160), self.font, self.font_scale,(255,0,255),2,cv2.LINE_AA)
            cv2.circle(img, (int(img.shape[0]/2),int(img.shape[1]/2)), pix_off_tot, (255, 255, 255), 2)
            cv2.circle(img, (pix_off_x,pix_off_y), 10, (255, 255, 255), -1)
        cv2.imshow('doin stuff', img)
#        if cv2.waitKey(1) & 0xFF == ord('q'):
#            break
    
    def handle_quit(self, delay=10):
        """Quit the program if the user presses "Esc" or "q"."""
        key = cv2.waitKey(delay)
        c = chr(key & 255)
        if c in ['c', 'C']:
            self.trail = np.zeros((self.cam_height, self.cam_width, 3),
                                     np.uint8)
        if c in ['q', 'Q', chr(27)]:
            sys.exit(0)  
            
    def save_data(self):
        '''
        Check to see if directory for todays date exists, if not then create one
        and then save data
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
           
        df = pd.read_csv(f_name, header=None, index_col=None)
        n_cols=len(df.columns)
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
        header[9]=self.filter_mode
        header[10]=self.track_time
        header[11]=self.ss_eshim_x
        header[12]=self.ss_eshim_y
        df.columns = header
        df.to_csv(f_name, index=False)
        
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
        header[9]='filter_mode'
        header[10]='track_time'
        header[11]='eshim_x'
        header[12]='eshim_y'
        df.columns = header
        df.to_csv(f_name, index=False)
        
        #Save data as matlab .mat file for simulation
        sio.savemat(cwd+'/run'+str(run_number)+'.mat',
                    {'elapsed':self.data['elapsed'].values,
                     'imu_filt_x':self.data['imu_filt_x'].values,
                     'imu_filt_y':self.data['imu_filt_y'].values,
                     'imu_ang_x':self.data['imu_ang_x'].values,
                     'imu_ang_y':self.data['imu_ang_y'].values,
                     'imu_ang_z':self.data['imu_ang_z'].values,
                     'ang_x_track':self.data['ang_x_track'].values,
                     'ang_y_track':self.data['ang_y_track'].values,
                     'ptu_cmd_x':self.data['ptu_cmd_x'].values,
                     'ptu_cmd_y':self.data['ptu_cmd_y'].values,
                     'kpx':self.pid_x.Kp,
                     'kpy':self.pid_y.Kp,
                     'kix':self.pid_x.Ki,
                     'kiy':self.pid_y.Ki,
                     'kdx':self.pid_x.Kd,
                     'kdy':self.pid_y.Kd,
                     'hz':self.hz,
                     'track_mode':self.track_mode,
                     'filter_mode':self.filter_mode,
                     'track_time':self.track_time,
                     'ss_eshim_x':self.ss_eshim_x,
                     'ss_eshim_y':self.ss_eshim_y})
    
    def filter_butter(self,data):
        b = signal.firwin(self.filter_win, 0.004)
        z = signal.lfilter_zi(b, 1)
        butter = zeros(data.size)
        for i, x in enumerate(data):
            butter[i], z = signal.lfilter(b, 1, [x], zi=z)
        return butter      
        
           
    def run(self):
        '''
        Start tracking loop
        '''
        #Initialize PTU
        self.setup_ptu()
        
        #Setup display window
        if self.show_display:     
            self.setup_display()
        
        #Reference time for elapsed tracking time
        self.t_start = time.time()   
        self.cnt = 0
        
        while True:
            #Time reference to ensure tracking operates at approximately set data rate
            self.t0 = time.time()
                
            try:
                with open(self.read_dir) as f:
                    data = f.read()
                    self.ang_x_track = float(data.split(',')[0])#+random.gauss(0,0.005)/10
                    self.ang_y_track = float(data.split(',')[1])#+random.gauss(0,0.005)/10
            except:
                print('Could not read camera degree offsets from file')
                self.ang_x_track = np.nan
                self.ang_y_track = np.nan
            print('sun center at ',self.ang_x_track,self.ang_y_track)
            
            #Filter Step 2: apply desired filter (filter mode) to data in filter window
            if self.filter_mode == 1:
                #Don't need to filter the SS data any more
                #Set imu filter values to nan if not using filtered data
                self.imu_ang_r = self.imu.grab_ang_r()
                self.imu_filt_x = self.imu_ang_r.x
                self.imu_filt_y = self.imu_ang_r.y
             
            if self.filter_mode > 1:
                #only start filtering after enough samples (size of filter window) have been recorded
                if self.cnt < self.filter_win:
                    #Set imu filter values to raw values until have enough samples to filter
                    self.imu_ang_r = self.imu.grab_ang_r()
                    self.imu_filt_x = self.imu_ang_r.x
                    self.imu_filt_y = self.imu_ang_r.y             
                else:
                    #Create array of past data with number of elements=filter window size (self.filter_win)     
                    ss_raw_x = np.array(self.data['ang_x_track'][-(self.filter_win-1):].tolist() + [self.ang_x_track])
                    ss_raw_y = np.array(self.data['ang_x_track'][-(self.filter_win-1):].tolist() + [self.ang_y_track])
                    
                    #Collect IMU angular rates (current data stored within IMU class)
                    self.imu_ang_r = imu.grab_ang_r()
#                    imu_raw_x = (180./np.pi)*np.array(self.data['imu_ang_z'][-(self.filter_win-1):].tolist() + [(180./np.pi)*self.imu_ang_r.z])
#                    imu_raw_y = (180./np.pi)*np.array(self.data['imu_ang_y'][-(self.filter_win-1):].tolist() + [(180./np.pi)*self.imu_ang_r.y])
                    
                    imu_raw_x = np.array(self.data['imu_ang_z'][-(self.filter_win-1):].tolist() + [self.imu_ang_r.z])
                    imu_raw_y = np.array(self.data['imu_ang_y'][-(self.filter_win-1):].tolist() + [self.imu_ang_r.y])
                    
                    if self.filter_mode == 2:  #Rolling mean (just take mean of samples in filter window)
                        self.ss_filt_x = np.nanmean(ss_raw_x)
                        self.ss_filt_y = np.nanmean(ss_raw_y)
                        self.imu_filt_x = np.nanmean(imu_raw_x)
                        self.imu_filt_y = np.nanmean(imu_raw_y) 
                        self.ang_x_track = self.ss_filt_x
                        self.ang_y_track = self.ss_filt_x
                        
                    if self.filter_mode == 3:  #Apply Butterworth filter to samples in filter window
                        self.ss_filt_x = self.butter(ss_raw_x)
                        self.ss_filt_y = self.butter(ss_raw_y)
                        self.imu_filt_x = self.butter(imu_raw_x)
                        self.imu_filt_y = self.butter(imu_raw_y)
                        self.ang_x_track = self.ss_filt_x[-1]
                        self.ang_y_track = self.ss_filt_x[-1]
      
            if self.track_mode == 1:   #PTU position-command mode: Simple PID control of ss offset
                try:
                    self.pid_pos(self.ang_x_track,self.ang_y_track)  #Generate PID offset control outputs
                    self.ptu_cmd_x = self.pid_out_x*self.pid_x.deg2pos  #PTU position command_x = PID_x control output
                    self.ptu_cmd_y = self.pid_out_y*self.pid_y.deg2pos  #PTU position command_y = PID_y control output
                    if self.track_x:
                        self.ptu.cmd('po'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                        time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
                    if self.track_y:
                        self.ptu.cmd('to'+str(self.ptu_cmd_y)+' ')    #Send PTU command to pan axis
                except:
                    self.ptu_cmd_x = np.nan
                    self.ptu_cmd_y = np.nan 
            
            if self.track_mode == 2:   #PTU velocity-command mode: -imu velocity + PID control of ss position
#                try:
                self.pid_pos(self.ang_x_track,self.ang_y_track)   #Generate PID offset control outputs
                self.ptu_cmd_x = -self.imu_filt_x + self.pid_out_x*self.pid_x.deg2pos  #PTU velocity x = -imu_ang_z + PID control output
                self.ptu_cmd_y = -self.imu_filt_y + self.pid_out_y*self.pid_y.deg2pos  #PTU velocity y = -imu_ang_y + PID control output
                if self.track_x:
                    self.ptu.cmd('ps'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                    print('ps'+str(self.ptu_cmd_x)+' ')
                    time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
                if self.track_y:
                    self.ptu.cmd('ts'+str(self.ptu_cmd_y)+' ')    #Send PTU command to pan axis
                    print('ts'+str(self.ptu_cmd_y)+' ')
#                except:
#                    self.ptu_cmd_x = np.nan
#                    self.ptu_cmd_y = np.nan 
#                    print('ptu command failed tracking mode 2')
                    
            if self.track_mode == 3:   #PTU velocity-command mode: -imu velocity - derivative of ss_offset + PID control of ss position
                try:
                    self.pid_pos(self.ang_x_track,self.ang_y_track)   #Generate PID offset control outputs
                    try:
                        self.ss_vel_x = (self.ss_filt_x[-1] - self.ss_filt_x[-3])/(2*(self.dt))  #Calculate ss x position derivative (central diff) from filtered ss data
                        self.ss_vel_y = (self.ss_filt_y[-1] - self.ss_filt_y[-3])/(2*(self.dt))  #Calculate ss y position derivative (central diff) from filtered ss data
                    except:
                        if self.filter_win < 3:
                            print('Filter window size needs to be >= 3 for tracking mode 3 (need three points to calculate derivative accurately)')
                        print('Cannot calculate SS velocity')
                        
                    self.ptu_cmd_x = -self.imu_filt_x[-1] - self.ss_vel_x + self.pid_out_x*self.pid_x.deg2pos  #PTU velocity x = -imu_ang_z + PID control output
                    self.ptu_cmd_y = -self.imu_filt_y[-1] - self.ss_vel_y + self.pid_out_y*self.pid_y.deg2pos  #PTU velocity y = -imu_ang_y + PID control output
                    if self.track_x:
                        self.ptu.cmd('ps'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                        time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
                    if self.track_y:
                        self.ptu.cmd('ts'+str(self.ptu_cmd_y)+' ')    #Send PTU command to pan axis
                except:
                    self.ptu_cmd_x = np.nan
                    self.ptu_cmd_y = np.nan
                           
            #Record time elapsed from start of tracking loop
            self.elapsed = time.time() - self.t_start
            self.d_time = datetime.now()
            if self.cnt > 1:
                self.dt = self.elapsed - self.data['elapsed'][self.cnt-1]
            
            #Update display
            if self.show_display == True:
                self.update_display()
            if self.cnt <= self.filter_win:
                self.imu_ang_r = self.imu.grab_ang_r()
                self.imu_filt_x = np.nan
                self.imu_filt_y = np.nan
            self.imu_accel=self.imu.grab_accel()
            self.imu_ypr=self.imu.grab_ypr()
            self.imu_mag=self.imu.grab_mag()
            data_add = [self.ang_x_track,
                        self.ang_y_track,
#                        ang_x[0],
#                        ang_y[0],
#                        ang_x[1],
#                        ang_y[1],
#                        ang_x[2],
#                        ang_y[2],
                        self.ptu_cmd_x,
                        self.ptu_cmd_y,
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
                        self.imu_filt_y,
                        self.elapsed,
                        ]
            self.data.loc[self.d_time] = data_add
            
            self.cnt+=1
            
            #Maintain desired data rate
            t_diff = time.time() - self.t0
            if self.delay - t_diff > 0:
                time.sleep(self.delay - t_diff)
            
            #Check to see if tracking time has expired
            if (time.time() - self.t_start) > self.track_time:
                #Stop PTU from moving after tracking completes
                try:
                    self.ptu.cmd('ps0 ')
                    self.ptu.cmd('ts0 ')
                except:
                    print('Could not send PTU zero speed command, watch your toes!')
                print('Tracking complete, thanks for playing!')
                return
                      
            if self.show_display:
                self.handle_quit()
                
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
    
    parser.add_argument('-tm','--track_mode',
                        default=2,
                        type=int,
                        help='Tracking mode')
    
    parser.add_argument('-fm','--filter_mode',
                        default=2,
                        type=int,
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
                        default=240,
                        type=float,
                        help='Total time to track (seconds)')
    
    parser.add_argument('-hz','--hz',
                        default=5,
                        type=float,
                        help='Tracking frequency (hz)')
    
    parser.add_argument('-s','--save_dir',
                        default=cwd+'/testing/',
                        type=str,
                        help='Directory to save data to')
    
    parser.add_argument('-r','--read_dir',
                        default=cwd+'/Data.txt',
                        type=str,
                        help='Directory to read camera position from')
    
#    parser.add_argument('-h','--help',
#                        default=False,
#                        type=bool,
#                        help='Display help')

###### PID parameters ###############
    parser.add_argument('-kpx','--kpx',
                        default=12.53,
                        type=float,
                        help='Proportional gain x-axis')
    
    parser.add_argument('-kpy','--kpy',
                        default=-3.0,
                        type=float,
                        help='Proportional gain y-axis')
    
    parser.add_argument('-kdx','--kdx',
                        default=0.0,
                        type=float,
                        help='Derivative gain x-axis')
    
    parser.add_argument('-kdy','--kdy',
                        default=-0.5,
                        type=float,
                        help='Derivative gain y-axis')
    
    parser.add_argument('-kix','--kix',
                        default=0,
                        type=float,
                        help='Integral gain x-axis')
    
    parser.add_argument('-kiy','--kiy',
                        default=0.0,
                        type=float,
                        help='Integral gain y-axis')

    
######## Sun sensor parameters #################
    
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
    
    parser.add_argument('-ss1_ex','--ss1_eshim_x',
                        default=0.0,
                        type=float,
                        help='SS1 electronic shim x-axis')
    
    parser.add_argument('-ss2_ex','--ss2_eshim_x',
                        default=0.0,
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
                        default='COM5',
                        type=str,
                        help='IMU comm port')    
    
    parser.add_argument('-imu_b','--imu_baud_rate',
                        default=115200,
                        type=int,
                        help='IMU baud_rate')
    
    
###### PTU parameters ###########
    parser.add_argument('-ptu_c','--ptu_com_port',
                        default='COM6',
                        type=str,
                        help='IMU comm port')    
    
    parser.add_argument('-ptu_b','--ptu_baud_rate',
                        default=9600,
                        type=int,
                        help='IMU baud_rate')
    
    parser.add_argument('-ptu_d','--ptu_cmd_delay',
                        default=0.01,
                        type=float,
                        help='PTU command delay')
    
    parser.add_argument('-ptu_s','--ptu_step_size',
                        default='eighth',
                        type=str,
                        help='PTU step size')

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
    filter_mode = params.filter_mode  #default: 1 raw SS and IMU data
    ptu_offset_mode = params.ptu_offset_mode  #default: 0 no PTU offset prior to tracking
    
    #Show tracking display
    show_display = params.display
    print('show_display = ',show_display)
    
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
    
    #Define tracking/data collection parameters
    track_time= params.track_time #20  #number of seconds to capture data/track
    hz=params.hz #15      #data sample rate   
    cnt=0
    delay = 1.0/hz
    
    #Define directory to save data in
    save_dir = params.save_dir #'C:/git_repos/GLO/Tutorials/tracking/'
    read_dir = params.read_dir

    #Obtain ephemeris data
    ep = ephem.Observer()

    #Establish communication with sun sensor/s - store in a list
    ss=[SS(inst_id=params.ss1_inst_id,com_port=params.ss1_com_port,baudrate=params.ss1_baud_rate),
        SS(inst_id=params.ss2_inst_id,com_port=params.ss2_com_port,baudrate=params.ss2_baud_rate),
        SS(inst_id=params.ss3_inst_id,com_port=params.ss3_com_port,baudrate=params.ss3_baud_rate)]
    
    #List of sun sensors to read data from (reduce number of sensors to increase sampling rate)
    ss_read = [0]
    
    #List of sun sensors to use for tracking
    ss_track = []
    if params.ss1_track:
        ss_track.append(1)
    if params.ss2_track:
        ss_track.append(2)
    if params.ss3_track:
        ss_track.append(3)
    
#    ss_eshim_x = [-1.763, -1.547, -1.578]          #Specify electronic shims (x-dir) for sun sensors
#    ss_eshim_y = [-2.290, -2.377, -2.215]          #Specify electronic shims (y-dir) for sun sensors
    ss_eshim_x = [params.ss1_eshim_x,
                  params.ss2_eshim_x,
                  params.ss3_eshim_x]          #Specify electronic shims (x-dir) for sun sensors
    ss_eshim_y = [params.ss1_eshim_y,
                  params.ss2_eshim_y,
                  params.ss3_eshim_y]          #Specify electronic shims (y-dir) for sun sensors

    print('eshims_x',ss_eshim_x)
    print('eshims_y',ss_eshim_y)
    
    #Establish communication with IMU
    imu=IMU(com_port=params.imu_com_port,baudrate=params.imu_baud_rate)
    
    #Establish communication with PTU
    ptu_cmd_delay=params.ptu_cmd_delay #0.010
    ptu = PTU(com_port='COM6',baudrate=9600)
#    ptu = PTU(com_port=params.ptu_com_port,
#              baudrate=params.ptu_baud_rate,
#              cmd_delay=params.ptu_cmd_delay)
    #Set latitude, longitude and altitude to Blacksburg, VA for sun pointing
    ptu.lat, ptu.lon, ptu.alt = params.ptu_lat,params.ptu_lon,params.ptu_alt  #'37.205144','-80.417560', 634
    ptu.utc_off=params.ptu_utc_off #4   #Set UTC time offset of EST

    #Find the Sun and the moon from your location
#    lat,lon,alt='37.205144','-80.417560',634    #Blacksburg
#    utc_datetime = datetime.now()   #Use current time (can also set to custom datetime= '2018/5/7 16:04:56')
    ptu.ephem_point(ep,imu=imu,target='sun',init=False,ptu_cmd=False)
    ptu.ephem_point(ep,imu=imu,target='moon',init=False,ptu_cmd=False)
    
    #Microstep mode positions/degree ~ 23.4, so check to make sure PTU is in microstep mode, if not then set it
    if (ptu.pan_pdeg > 24) | (ptu.tilt_pdeg > 24):
        ptu.set_microstep()
        input('Press any key when PTU has completed calibration')
        
    if ptu_offset_mode == 1:
        #Command PTU to point at sun
        ptu.ephem_point(ep,imu=imu,target='sun',init=False)
    if ptu_offset_mode == 2:
        #Command PTU to point at moon
        ptu.ephem_point(ep,imu=imu,target='moon',init=False)

    #Set ptu=None if not using tracking to ensure PTU is not moved after initial offset
    if track_mode == 4:
        ptu.ptu.close()
        ptu=None
        print('Not tracking, so disconnecting from the PTU for safe measure')

    #Initiate PTU tracking
    ss_tracking = SS_tracking(ss,
                             ptu,
                             imu,
                             ss_read=ss_read,
                             ss_track=ss_track,
                             ss_eshim_x=ss_eshim_x,
                             ss_eshim_y=ss_eshim_y,
                             pid_x=pid_x,
                             pid_y=pid_y,
                             ptu_cmd_delay=ptu_cmd_delay,
                             track_mode=track_mode,
                             filter_mode=filter_mode,
                             filter_win=params.filter_win,
                             hz=hz,
                             track_time=track_time,
                             save_dir=save_dir,
                             show_display=show_display,
                             track_x=params.track_x,
                             track_y=params.track_y,
                             read_dir=read_dir
                             )
        
    print('Tracking with sun sensors',ss_track,'for',track_time,'seconds')
    
    #Begin PTU tracking
    ss_tracking.run()
    
    #Save data
    ss_tracking.save_data()
    
    #Grab data in dataframe
    df = ss_tracking.data
        
    #Close IMU connection
    try:
        imu.imu.disconnect()
    except:
        print('Could not disconnect from IMU')
    
    #Close PTU connection
    try:
        ptu.ptu.close()
    except:
        print('Could not disconnect from PTU')
    
    #Close sun sensor connections
    for i in range(len(ss)):
        try:
            ss[i].ss.serial.close() 
        except:
            print('could not close sun sensor',i)
        
    try:
    
        #Plot y_angle raw vs. filtered 
        x=df['elapsed']
        y1=df['imu_ang_z']*180./np.pi
        y2=df['imu_filt_x']*180./np.pi
#        y3=df['ss2_x_raw']
        y4=df['ang_x_track']
#        y5=df['ss2_x_raw']
        y6=df['ptu_cmd_x']*23/3600.
        
        plt.figure(1)
        plt.plot(x,y1,'o-',label='imu_ang_z')
        plt.xlabel('Time Elapsed (seconds)')
        plt.ylabel('Degrees')
        plt.title('X-Axis sensor data at '+str(hz)+'hz\n kp='+str(params.kpx)+' ki='+str(params.kix)+' kd='+str(params.kdx))
        plt.legend()
        
        #plt.figure(2)
       # plt.plot(x,y2,'o-',label='imu_filt_x')
        plt.plot(x,y4,'o-',label='ss2_ang_x_raw')
       # plt.plot(x,y4,'o-',label='filtered ss')
       # plt.plot(x,y6,'o-',label='ptu cmd x')
        plt.xlabel('Time Elapsed (seconds)')
        plt.ylabel('Degrees')
        #plt.title('Y-Axis sensor data at '+str(hz)+'hz\n kp='+str(params.kpy)+' ki='+str(params.kiy)+' kd='+str(params.kdy))
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