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
import argparse
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import cv2
import ephem
import os


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
                 ss_read=[1,2,3],
                 ss_track=[1,2,3],
                 ss_eshim_x=[0.0,0.0,0.0],
                 ss_eshim_y=[0.0,0.0,0.0],
                 pid_x=None,
                 pid_y=None,
                 ptu_cmd_delay=0.025,
                 track_mode=3,
                 filter_mode=1,
                 hz=20,
                 track_time=120,
                 save_dir = 'C:/git_repos/GLO/Tutorials/tracking/',
                 show_display=True,
                 screen_res = (1280,800)
                 ):
        
        #Initialize parameters
        self.ss = ss
        self.ptu = ptu
        self.ss_read = ss_read
        self.ss_track = ss_track
        self.ss_eshim_x = ss_eshim_x
        self.ss_eshim_y = ss_eshim_y
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.ptu_cmd_delay=ptu_cmd_delay
        self.track_mode=track_mode
        self.filter_mode=filter_mode
        self.hz=hz
        self.delay = 1.0/hz
        self.track_time=track_time
        self.save_dir = save_dir
        self.show_display = show_display
        self.screen_res = screen_res
        self.max_vel_x = 1.0
        self.max_vel_y = 1.0
    
        #Initialized dataframe to store data  
        self.data={}
        for i in ss_read:
            self.data['ss'+str(i)] = pd.DataFrame(columns=['ang_x_track',
                                                           'ang_y_track',
                                                           'ang_x_raw',
                                                           'ang_y_raw',
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
            #Absolute Velocity Mode
            if self.track_mode == 2:
                self.ptu.cmd('cv ')
                time.sleep(0.1)
                self.ptu.cmd('i ')
                time.sleep(0.1)
                self.ptu.cmd('ps0 ')
                time.sleep(0.1)
                self.ptu.cmd('ts0 ')
            #Differential Velocity Mode
            if self.track_mode == 3:
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
        
        #Save data to file   
        for i in range(len(self.ss_read)):
            print('saving ss',str(ss_read[i]),'tracking data to',self.save_dir+dir_date+'ss_track_ss'+str(ss_read[i])+'_'+file_time+'.csv')
            self.data['ss'+str(ss_read[i])].to_csv(self.save_dir+dir_date+'ss_track_ss'+str(ss_read[i])+'_'+file_time+'.csv',index_label='time')
               
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
        
        ang_x = np.zeros(len(self.ss),dtype=float)   
        ang_y = np.zeros(len(self.ss),dtype=float)
        
        while True:
            #Time reference to ensure tracking operates at approximately set data rate
            self.t0 = time.time()
            #Initialize numpy arrays to hold raw x and y offsets from all SS for filtering
            
            ang_x.fill(np.nan)
            ang_y.fill(np.nan)
            
            #Collect Sun Sensor data
            for i in ss_read:    #Loop through all sun sensors
                self.ss[i-1].read_data_all()    #Read all data from sun sensor using SS class      
                if i in self.ss_track:   #Only include x and y SS offsets if included in ss_track
                    ang_x[i-1] = self.ss[i-1].ang_x_raw + self.ss_eshim_x[i-1]
                    ang_y[i-1] = self.ss[i-1].ang_y_raw + self.ss_eshim_y[i-1]
        
            #Filter Sun Sensor data:
            #  currently this is just taking the mean of all sun sensors listed in ss_track
            self.ang_x_track = np.nanmean(ang_x)
            self.ang_y_track = np.nanmean(ang_y)
                
            #Record time elapsed from start of tracking loop
            self.elapsed = time.time() - self.t_start
            self.dt = self.elapsed - self.data['elapsed'][self.cnt-1]
            
            #Calculate sun sensor velocity using last three ss samples
            if self.cnt > 1:
                self.ss_vel_x = (self.ang_x_track - self.data['ss_track_x'][self.cnt-2])/(2*(self.dt))
                self.ss_vel_y = (self.ang_y_track - self.data['ss_track_y'][self.cnt-2])/(2*(self.dt))
                    
            try:
                if self.ang_x_track != 0.0:
                    self.vel_off_x = self.pid_x.GenOut(self.ang_x_track)  #generate x-axis control output in degrees
                    #enforce maximum speed in x-direction corrections
                    if self.vel_off_x > self.max_vel_x:
                        self.vel_off_x = self.max_vel_x
                    if self.vel_off_x < -self.max_vel_x:
                        self.vel_off_x = -self.max_vel_x
                    self.ptu_cmd_x = -(self.imu_ang_z + self.ss_vel_x + self.vel_off_x)*self.pid_x.deg2pos  #convert to PTU positions
                    self.ptu.cmd('ps'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                    time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
                else:
                    self.outv_x = np.nan
                    self.ptu_cmd_x = np.nan
                
                if self.ang_y_track != 0.0:
                    self.vel_off_y = self.pid_y.GenOut(self.ang_y_track)  #generate y-axis control output in degrees
                    #enforce maximum speed in y-direction corrections
                    if self.vel_off_y > self.max_vel_y:
                        self.vel_off_y = self.max_vel_y
                    if self.vel_off_y < -self.max_vel_y:
                        self.vel_off_y = -self.max_vel_y
                    self.ptu_cmd_y = -(self.ss_vel_y + self.vel_off_y)*self.pid_y.deg2pos  #convert to PTU positions
                    self.ptu.cmd('ts'+str(self.ptu_cmd_y)+' ')    #Send PTU command to tilt axis
                    time.sleep(self.ptu_cmd_delay)   #sleep for set delay between different ptu axis commands
                else:
                    self.outv_y = np.nan
                    self.ptu_cmd_y = np.nan  
            except:
                print('PTU velocity tracking command failed')
                self.ptu_cmd_x = np.nan 
                self.ptu_cmd_y = np.nan     

            else:
                self.ptu_cmd_x=np.nan
                self.ptu_cmd_y=np.nan
            
            #Update display
            if self.show_display == True:
                self.update_display()
 
            for i in self.ss_read:    
                data_add = [self.ang_x_track,
                            self.ang_y_track,
                            ang_x[i-1],
                            ang_y[i-1],
                            self.ptu_cmd_x,
                            self.ptu_cmd_y,
                            self.imu.accel.x,
                            self.imu.accel.y,
                            self.imu.accel.z,
                            self.imu.ang_r.x,
                            self.imu.ang_r.y,
                            self.imu.ang_r.z,
                            self.imu.mag.x,
                            self.imu.mag.y,
                            self.imu.mag.z,
                            self.imu.ypr.x,
                            self.imu.ypr.y,
                            self.imu.ypr.z,                            
                            self.elapsed,
                            ]
                self.data['ss'+str(i)].loc[self.d_time] = data_add
            
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
    parser.add_argument('-tm','--track_mode',
                        default=1,
                        type=int,
                        help='Tracking mode')
    
    parser.add_argument('-fm','--filter_mode',
                        default=1,
                        type=int,
                        help='Filter mode')
    
    parser.add_argument('-pm','--ptu_offset_mode',
                        default=0,
                        type=int,
                        help='PTU offset mode')
    
    parser.add_argument('-m','--man_config',
                        default=True,
                        type=bool,
                        help='Manual config mode')

    parser.add_argument('-d','--display',
                        default='False',
                        type=str,
                        help='show display')
    
    parser.add_argument('-t','--track_time',
                        default=10,
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

###### PID parameters ###############
    parser.add_argument('-kpx','--kpx',
                        default=0.3,
                        type=float,
                        help='Proportional gain x-axis')
    
    parser.add_argument('-kpy','--kpy',
                        default=-0.44*0,
                        type=float,
                        help='Proportional gain y-axis')
    
    parser.add_argument('-kdx','--kdx',
                        default=0.3,
                        type=float,
                        help='Derivative gain x-axis')
    
    parser.add_argument('-kdy','--kdy',
                        default=-0.3*0,
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
    
    parser.add_argument('-ss1','--ss1_track',
                        default=True,
                        type=bool,
                        help='Track with SS1 (True/False)')
    
    parser.add_argument('-ss2','--ss2_track',
                        default=True,
                        type=bool,
                        help='Track with SS2 (True/False)')
    
    parser.add_argument('-ss3','--ss3_track',
                        default=True,
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
                        default='COM6',
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
    parser.add_argument('-ptu_c','--ptu_com_port',
                        default='COM5',
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
    #Define Default Modes  
    default_track_mode = 4      #no tracking
    default_filter_mode = 1     #raw SS and IMU data
    default_ptu_offset_mode = 0 #no PTU offset prior to tracking

    
    manual_config = params.man_config    #Set to True to manually configure settings
    show_display = params.display
    
    print(show_display)
    
    #Define PID control gains
    #pan-axis gains
    kp_x=params.kpx #0.44
    ki_x=params.kix #0.05*0
    kd_x=params.kdx #0.3
    
    #tilt-axis gains
    kp_y=params.kpy #-0.44
    ki_y=params.kiy #0.01*0
    kd_y=params.kdy #-0.3
    
    print('kpid_x',kp_x,ki_x,kd_x)
    print('kpid_y',kp_y,ki_y,kd_y)
    
    
    
    #Define data collection parameters
    track_time= params.track_time #20  #number of seconds to capture data/track
    hz=params.hz #15      #data sample rate   
    cnt=0
    delay = 1.0/hz
    
    #Define directory to save data in
    save_dir = params.save_dir #'C:/git_repos/GLO/Tutorials/tracking/'

    #Obtain ephemeris data
    ep = ephem.Observer()

    #Establish communication with sun sensor/s - store in a list
    ss=[SS(inst_id=1,com_port='COM4',baudrate=115200),
        SS(inst_id=2,com_port='COM4',baudrate=115200),
        SS(inst_id=3,com_port='COM4',baudrate=115200)]
    
    #List of sun sensors to read data from (reduce number of sensors to increase sampling rate)
    ss_read = [1,2,3]
    
    #List of sun sensors to use for tracking
    ss_track = []
    if params.ss1_track:
        ss_track.append(1)
    if params.ss2_track:
        ss_track.append(2)
    if params.ss1_track:
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
    imu=IMU(com_port='COM7',baudrate=115200)
    
    #Establish communication with PTU
    ptu_cmd_delay=params.ptu_cmd_delay #0.010
    ptu = PTU(com_port=params.ptu_com_port,
              baudrate=params.ptu_baud_rate,
              cmd_delay=params.ptu_cmd_delay)
    #Set latitude, longitude and altitude to Blacksburg, VA for sun pointing
    ptu.lat, ptu.lon, ptu.alt = params.ptu_lat,params.ptu_lon,params.ptu_alt  #'37.205144','-80.417560', 634
    ptu.utc_off=params.ptu_utc_off #4   #Set UTC time offset of EST

    #Find the Sun and the moon from your location
#    lat,lon,alt='37.205144','-80.417560',634    #Blacksburg
#    utc_datetime = datetime.now()   #Use current time (can also set to custom datetime= '2018/5/7 16:04:56')
    ptu.ephem_point(ep,imu=imu,target='sun',init=False,ptu_cmd=False)
    ptu.ephem_point(ep,imu=imu,target='moon',init=False,ptu_cmd=False)
    
    if manual_config == True:
        ptu_micro = int(input('Set PTU to microstep mode?:\n'+
                               '0: No\n'+
                               '1: Yes\n'+
                               '>>> '))
        if ptu_micro == 1:
            ptu.set_microstep()
            input('Press any key when PTU has completed calibration')
            
        track_mode = int(input('Select PTU Tracking Mode:\n'+
                               '1: PID Position Control\n'+
                               '2: PID Absolute Velocity Control\n'+
                               '3: PID Velocity Derivative Control\n'+
                               '4: No tracking - Read Sun Sensor Data Only\n'+
                               '5: Ephemeris Tracking: Stationary platform\n'+
                               '6: Ephemeris Tracking: Moving platform (need GPS sensor)\n'+
                               '>>> '))
        
        if track_mode !=4:
            filter_mode = int(input('Select Sun Sensor Filtering Mode:\n'+
                                    '1: Raw data: Use mean of raw data from all tracking sun sensors\n'+
                                    '2: Filtered data: Use mean of filtered data from all tracking sun sensors\n'+
                                    '3: Kalman Filter: probably not implemented yet...\n'+
                                    '>>> '))
        else:
            filter_mode = default_filter_mode
        
        ptu_offset_mode = int(input('Select PTU offset mode:\n'+
                       '0: No Pointing Offset\n'+
                       '1: Point PTU at Sun\n'+
                       '2: Point PTU at Moon\n'+
                       '>>> '))  
        
        if ptu_offset_mode == 1:
            #Command PTU to point at sun
            ptu.ephem_point(ep,imu=imu,target='sun',init=False)
        if ptu_offset_mode == 2:
            #Command PTU to point at moon
            ptu.ephem_point(ep,imu=imu,target='moon',init=False)
        
    else:
        track_mode = default_track_mode
        filter_mode = default_filter_mode
        ptu_offset_mode = default_ptu_offset_mode
    
    
    #Initiate PID control loop
    pid_x= PID(step_size='eighth')     #pid_x will control azimuth ptu motor (assuming orientation of ss is correct)
    pid_x.SetKp(kp_x)
    pid_x.SetKi(ki_x)
    pid_x.SetKd(kd_x)
    
    pid_y= PID(step_size='eighth')     #pid_y will control azimuth ptu motor (assuming orientation of ss is correct)
    pid_y.SetKp(kp_y)
    pid_y.SetKi(ki_y)
    pid_y.SetKd(kd_y)   

    #Set ptu=None if not using tracking to ensure PTU is not moved after initial offset
    if track_mode == 4:
        ptu.ptu.close()
        ptu=None
        print('Not tracking, so disconnecting from the PTU for safe measure')

    #Initiate PTU tracking
    ss_tracking = SS_tracking(ss,
                            ptu,
                            ss_read=ss_read,
                            ss_track=ss_track,
                            ss_eshim_x=ss_eshim_x,
                            ss_eshim_y=ss_eshim_y,
                            pid_x=pid_x,
                            pid_y=pid_y,
                            ptu_cmd_delay=ptu_cmd_delay,
                            track_mode=track_mode,
                            filter_mode=filter_mode,
                            hz=hz,
                            track_time=track_time,
                            save_dir=save_dir,
                            show_display=show_display
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
#        df['ss2'].plot(x='elapsed',y=['ang_x_filt','ang_x_raw'],grid=True,
#                marker='o',markersize=3)
        
        for i in range(len(ss_read)):
            ss_num = str(ss_read[i]) 
            x=df['ss'+ss_num]['elapsed']
            y1=df['ss'+ss_num]['ang_x_raw']+ss_eshim_x[i]
            y2=df['ss'+ss_num]['ang_y_raw']+ss_eshim_y[i]
            
            plt.figure(1)
            plt.plot(x,y1,'o-',label='ss'+ss_num+'_ang_x_raw')
            plt.xlabel('Time Elapsed (seconds)')
            plt.ylabel('Degrees')
            plt.title('X-Axis sensor data at '+str(hz)+'hz\n kp='+str(kp_x)+' ki='+str(ki_x)+' kd='+str(kd_x))
            plt.legend()
            
            plt.figure(2)
            plt.plot(x,y2,'o-',label='ss'+ss_num+'_ang_x_raw')
            plt.xlabel('Time Elapsed (seconds)')
            plt.ylabel('Degrees')
            plt.title('Y-Axis sensor data at '+str(hz)+'hz\n kp='+str(kp_x)+' ki='+str(ki_x)+' kd='+str(kd_x))
            plt.legend()
    except:
        print('Failed to plot data')