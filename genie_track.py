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
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import cv2
import ephem
import os
import socket


class Genie_tracking:
    '''
    PID tracking of the sun using a FLIR Pan Tilt Unit and feedback from Teledyne Dalsa Genie Camera
    The camera sun tracking algorithm is run with a separate C++ code and constantly
    sends the x,y offset values to a UDP port. The x,y offsets correspond to the 
    number of degrees off the center of the sun is from the center pixel of the camera.
    
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
                 screen_res = (1280,800),
                 UDP_IP,
                 UDP_PORT
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
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        
        #Initialized dataframe to store data  
        #Do not save sun sensor data if ss_read list is empty
        if ss_read == None:
            self.data = pd.DataFrame(columns=['ang_x_track',
                                              'ang_y_track',
                                              'ptu_cmd_x',
                                              'ptu_cmd_y',
                                              'elapsed'])
        else:
            self.data={}
            for i in ss_read:
                self.data['ss'+str(i)] = pd.DataFrame(columns=['ang_x_track',
                                                               'ang_y_track',
                                                               'ang_x_filt',
                                                               'ang_y_filt',
                                                               'ang_x_raw',
                                                               'ang_y_raw',
                                                               'watts',
                                                               'temp',
                                                               'ptu_cmd_x',
                                                               'ptu_cmd_y',
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
            
        #Initialize UDP Socket to capture camera data (x,y sun center pixels)
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.sock.bind((UDP_IP, UDP_PORT))
        
    def track_pos(self):
        '''
        Track using position control
            Converts SS PID error signal into a PTU position offset command 
        '''
        #If SS angle offsets are within bounds, generate PID error signal and ptu command
        try:
            if (self.ang_x_track > -5) & (self.ang_x_track < 5):
                self.outv_x = self.pid_x.GenOut(self.ang_x)  #generate x-axis control output in degrees
                self.ptu_cmd_x = self.outv_x*self.pid_x.deg2pos  #convert to PTU positions
                self.ptu.cmd('po'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
            else:
                self.outv_x = np.nan
                self.ptu_cmd_x = np.nan
            
            if (self.ang_y_track > -5) & (self.ang_y_track < 5):
                self.outv_y = self.pid_x.GenOut(self.ang_y)  #generate y-axis control output in degrees
                self.ptu_cmd_y = self.outv_y*self.pid_y.deg2pos  #convert to PTU positions
                self.ptu.cmd('to'+str(self.ptu_cmd_y)+' ')    #Send PTU command to tilt axis
                time.sleep(self.ptu_cmd_delay)
            else:
                self.outv_y = np.nan
                self.ptu_cmd_y = np.nan 
        except:
            print('PTU position tracking command failed')
            self.ptu_cmd_x = np.nan 
            self.ptu_cmd_y = np.nan 
            
        
    def track_vel(self):
        '''
        Track using velocity control
            Converts SS PID error signal into a PTU absolute velocity offset command 
        '''
        #If SS angle offsets are within bounds, generate PID error signal and ptu command
        try:
            if (self.ang_x_track > -5) & (self.ang_x_track < 5):
                self.outv_x = self.pid_x.GenOut(self.ang_x)  #generate x-axis control output in degrees
                self.ptu_cmd_x = self.outv_x*self.pid_x.deg2pos  #convert to PTU positions
                self.ptu.cmd('ps'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
            else:
                self.outv_x = np.nan
                self.ptu_cmd_x = np.nan
            
            if (self.ang_y_track > -5) & (self.ang_y_track < 5):
                self.outv_y = self.pid_x.GenOut(self.ang_y)  #generate y-axis control output in degrees
                self.ptu_cmd_y = self.outv_y*self.pid_y.deg2pos  #convert to PTU positions
                self.ptu.cmd('ts'+str(self.ptu_cmd_y)+' ')    #Send PTU command to tilt axis
                time.sleep(self.ptu_cmd_delay)   #sleep for set delay between different ptu axis commands
            else:
                self.outv_y = np.nan
                self.ptu_cmd_y = np.nan  
        except:
            print('PTU velocity tracking command failed')
            self.ptu_cmd_x = np.nan 
            self.ptu_cmd_y = np.nan 
        
    def track_dv(self):
        '''
        Track using differential velocity control
            Converts SS PID error signal into a PTU differential velocity offset command 
        '''
        #If SS angle offsets are within bounds, generate PID error signal and ptu command
        try:
            if (self.ang_x_track > -5) & (self.ang_x_track < 5):
                self.outv_x = self.pid_x.GenOut(self.ang_x_track)  #generate x-axis control output in degrees
                self.ptu_cmd_x = self.spd_last_x + self.outv_x*self.pid_x.deg2pos  #convert to PTU positions
                self.spd_last_x = self.ptu_cmd_x
                self.ptu.cmd('ps'+str(self.ptu_cmd_x)+' ')    #Send PTU command to pan axis
                time.sleep(self.ptu_cmd_delay)    #allow small delay between PTU commands
            else:
                self.outv_x = np.nan
                self.ptu_cmd_x = np.nan
            
            if (self.ang_y_track > -5) & (self.ang_y_track < 5):
                self.outv_y = self.pid_y.GenOut(self.ang_y_track)  #generate y-axis control output in degrees
                self.ptu_cmd_y = self.spd_last_y - self.outv_y*self.pid_y.deg2pos  #convert to PTU positions
                self.spd_last_y = self.ptu_cmd_y
                self.ptu.cmd('ts'+str(self.ptu_cmd_y)+' ')    #Send PTU command to tilt axis
                time.sleep(self.ptu_cmd_delay)
            else:
                self.outv_y = np.nan
                self.ptu_cmd_y = np.nan   
        except:
            print('PTU differential velocity tracking command failed')
            self.ptu_cmd_x = np.nan 
            self.ptu_cmd_y = np.nan 
        
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
        if ss_read == None:
            print('saving camera tracking data to',self.save_dir+dir_date+'cam_genie_track_'+file_time+'.csv')
            self.data.to_csv(self.save_dir+dir_date+'cam_genie_track_'+file_time+'.csv',index_label='time')
        else:
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
        
        while True:
            #Time reference to ensure tracking operates at approximately set data rate
            self.t0 = time.time()
            #Initialize numpy arrays to hold raw x and y offsets from all SS for filtering
            ang_x = np.zeros(len(self.ss),dtype=float)   
            ang_y = np.zeros(len(self.ss),dtype=float)
            ang_x.fill(np.nan)
            ang_y.fill(np.nan)
            
            #Do not read sun sensor data if ss_read list is empty
            if ss_read != None:
                #Collect Sun Sensor data
                for i in ss_read:    #Loop through all sun sensors
                    self.ss[i-1].read_data_all()    #Read all data from sun sensor using SS class      
            
            #Read x and y camera degree offsets corresponding to center of the sun
            data, addr = self.sock.recvfrom(1024) # buffer size is 1024 bytes
            self.ang_x_track = float(data.encode().split(',')[0])
            self.ang_y_track = float(data.encode().split(',')[1])
            print('sun center at ',self.ang_x_track,self.ang_y_track)
            
            #Feed into PID and create appropriate PTU command
            if self.track_mode == 1:
                self.track_pos()
            elif self.track_mode == 2:
                self.track_vel()
            elif self.track_mode == 3:
                self.track_dv()
            else:
                self.ptu_cmd_x=np.nan
                self.ptu_cmd_y=np.nan
                
            #print('PTU pan command:,',self.ptu_cmd_x)
            #print('PTU tilt command:,',self.ptu_cmd_y)
            
            #Update display
            if self.show_display == True:
                self.update_display()
            
            #Record date/time and add row to dataframe
            self.elapsed = time.time() - self.t_start
            self.d_time = datetime.now()
            
            #Do not save sun sensor data if ss_read list is empty
            if self.ss_read == None:
                data_add = [self.ang_x_track,
                            self.ang_y_track,
                            self.ptu_cmd_x,
                            self.ptu_cmd_y,
                            self.elapsed]
                self.data.loc[self.d_time] = data_add     
            else:
                for i in self.ss_read:    
                    data_add = [self.ang_x_track,
                                self.ang_y_track,
                                self.ss[i-1].ang_x_filt,
                                self.ss[i-1].ang_y_filt,
                                self.ss[i-1].ang_x_raw,
                                self.ss[i-1].ang_y_raw,
                                self.ss[i-1].watts,                         
                                self.ss[i-1].temp,
                                self.ptu_cmd_x,
                                self.ptu_cmd_y,
                                self.elapsed]
                    self.data['ss'+str(i)].loc[self.d_time] = data_add
            
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
    
    manual_config = True             #Set to True to manually configure settings
    show_display = True
    
    #UDP port to capture camera x,y pixel data
    UDP_IP = "127.0.0.1"
    UDP_PORT = 8080
    
    #Define PID control gains
    #pan-axis gains
    kp_x=0.3
    ki_x=0.05*0
    kd_x=0.3
    
    #tilt-axis gains
    kp_y=-0.3
    ki_y=0.01*0
    kd_y=-0.3
    
    #Define data collection parameters
    track_time=5  #number of seconds to capture data/track
    hz=10      #data sample rate   
    cnt=0
    delay = 1.0/hz
    
    #Define directory to save data in
    save_dir = 'C:/git_repos/GLO/Tutorials/tracking/'

    #Obtain ephemeris data
    ep = ephem.Observer()

    #Establish communication with sun sensor/s - store in a list
    ss=[SS(inst_id=1,com_port='COM4',baudrate=115200),
        SS(inst_id=2,com_port='COM4',baudrate=115200),
        SS(inst_id=3,com_port='COM4',baudrate=115200)]
    
    #List of sun sensors to read data from
    ss_read = [1,3]
    
    #List of sun sensors to use for tracking
    ss_track = [1,3]
    
#    ss_eshim_x = [-1.763, -1.547, -1.578]          #Specify electronic shims (x-dir) for sun sensors
#    ss_eshim_y = [-2.290, -2.377, -2.215]          #Specify electronic shims (y-dir) for sun sensors
    ss_eshim_x = [0.0,0.0,0.0]          #Specify electronic shims (x-dir) for sun sensors
    ss_eshim_y = [0.0,0.0,0.0]          #Specify electronic shims (y-dir) for sun sensors

    #Establish communication with IMU
    imu=IMU(com_port='COM7',baudrate=115200)
    
    #Establish communication with PTU
    ptu_cmd_delay=0.025
    ptu = PTU(com_port='COM5',baudrate=9600,cmd_delay=ptu_cmd_delay)
    #Set latitude, longitude and altitude to Blacksburg, VA for sun pointing
    ptu.lat, ptu.lon, ptu.alt = '37.205144','-80.417560', 634
    ptu.utc_off=4   #Set UTC time offset of EST

    #Find the Sun and the moon from your location
    lat,lon,alt='37.205144','-80.417560',634    #Blacksburg
    utc_datetime = datetime.now()   #Use current time (can also set to custom datetime= '2018/5/7 16:04:56')
    ptu.ephem_point(ep,imu=imu,target='sun',init=False,ptu_cmd=False)
    ptu.ephem_point(ep,imu=imu,target='moon',init=False,ptu_cmd=False)
    
    #Define Default Modes  
    default_track_mode = 4
    default_filter_mode = 1
    default_ptu_offset_mode = 0
    
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
    genie_tracking = Genie_tracking(ss,
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
                        show_display=show_display,
                        UDP_IP,
                        UDP_PORT
                        )
    
    print('Tracking with genie camera for',track_time,'seconds')
    
    #Begin PTU tracking
    genie_tracking.run()
    
    #Save data
    genie_tracking.save_data()
    
    #Grab data in dataframe
    df = genie_tracking.data
        
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
        if ss_read == None:
                x=df['elapsed']
                y1=df['ang_x_track']
                y2=df['ang_y_track']
                
                plt.figure(1)
                plt.plot(x,y1,'o-',label='ang_x_track')
                plt.xlabel('Time Elapsed (seconds)')
                plt.ylabel('Degrees')
                plt.title('X-Axis sensor data at '+str(hz)+'hz\n kp='+str(kp_x)+' ki='+str(ki_x)+' kd='+str(kd_x))
                plt.legend()
                
                plt.figure(2)
                plt.plot(x,y2,'o-',label='ang_y_track')
                plt.xlabel('Time Elapsed (seconds)')
                plt.ylabel('Degrees')
                plt.title('Y-Axis sensor data at '+str(hz)+'hz\n kp='+str(kp_x)+' ki='+str(ki_x)+' kd='+str(kd_x))
                plt.legend() 
        else:
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
    #    
    #    #Plot difference between every frame
    #    df.plot.scatter(x='time_elapsed',y='diff',grid=True)
    #    plt.xlabel('Time Elapsed (seconds)')
    #    plt.ylabel('Seconds between reads')
    #    plt.title('Time elapsed between read at '+str(hz)+'hz ('+str(1/hz)+' sec)')
    except:
        print('Failed to plot data')