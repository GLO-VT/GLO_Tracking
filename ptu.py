# -*- coding: utf-8 -*-
"""
Created on Thu May 17 14:12:47 2018

@author: Bailey group PC
"""

import serial
import time
from datetime import datetime
import numpy as np
try:
    import ephem
except:
    print('Unable to import ephem, cannot command PTU to point at sun or moon')
try:
    from vnpy import EzAsyncData
except:
    print('Unable to import vnpy (VN100 IMU library), cannot use IMU for sun/moon pointing')

#import pandas as pd

class PTU:
    '''
    Class to connect to FLIR Pan Tilt Unit
    
    Inputs:
        com_port: serial com port for USB to 485-converter
        baudrate: (int) default baudrate = 19200, fastest baudrate = 115200
        cmd_delay: (int) delay (in seconds) to wait between ptu commands
    '''
    def __init__(self,com_port='COM5',baudrate=9600,cmd_delay=0.025):
        self.com_port = com_port
        self.baudrate = baudrate
        self.cmd_delay = cmd_delay
        
        self.lat = '37.205144'
        self.lon = '-80.417560'
        self.alt = 634
        self.utc_off = 4
        
        self.pan_pdeg_def = 38.89    #Corresponds to 92.5714 arc sec per position (half-step mode for ebay PTU)
        self.tilt_pdeg_def = 38.89   #Corresponds to 92.5714 arc sec per position (half-step mode for ebay PTU)
        
        #Establish communication with PTU
        try:
            self.ptu = serial.Serial(port=self.com_port,baudrate=self.baudrate)
            if self.ptu.isOpen():
                self.cmd('i ')    #Send this command to PTU to make it respond to commands
                print('Connected to PTU D300 Ebay (or not)')
        except:
            print('Could not connect to PTU')
        
        #Determine pan/tilt PTU resolution and calculate conversion factors
        try:
            self.pan_res = float(str(self.read('pr ')).split('*')[1].split(' seconds')[0])
            self.pan_pdeg = self.pan_res/3600
            print('PTU pan axis resolution =',self.pan_res,'seconds of arc per position or',round(1/self.pan_pdeg,2),'positions per degree')
            self.tilt_res = float(str(self.read('tr ')).split('*')[1].split(' seconds')[0])
            self.tilt_pdeg = self.tilt_res/3600
            print('PTU tilt axis resolution =',self.pan_res,'seconds of arc per position or',round(1/self.tilt_pdeg,2),'positions per degree')
        except:
            print('Could not determine resolution of PTU, using default of 92.5714 positions per degree')
            self.pan_pdeg = self.pan_pdeg_def
            self.tilt_pdeg = self.tilt_pdeg_def
            
    def read(self,command,delay=0.5):
        '''
        Send ptu a command, and return the ptu response
        '''
        try:
            self.ptu.write(command.encode())
            time.sleep(delay)
            bytesToRead = self.ptu.inWaiting()
            ptu_out = self.ptu.read(bytesToRead)
            ptu_out = str(ptu_out).split(command)[1].split('\\r')[0]
            return ptu_out
        except:
            print('Could not read command from PTU')
            return
        
    def cmd(self,command):
        '''
        Send command to PTU
        '''
        self.ptu.write(command.encode())
        return
        
    def set_microstep(self):
        '''
        Set PTU to microstep mode and reset 
        ***WARNING: Make sure PTU range of motion is clear of stuff!!
        '''
        print('***WARNING***: Reseting PTU, make sure full range of PTU motion is clear of stuff!!!')
        ptu_reset = input('Are you sure you want to reset PTU?\n'+
                          '0: No\n'+
                          '1: Yes\n'+
                          '>>>')
        if int(ptu_reset) == 1:
            self.cmd('i ')
            time.sleep(0.1)
            self.cmd('ci ')
            time.sleep(0.5)
            self.cmd('wpe ')    #set pan-axis to eighth step mode
            time.sleep(2.0)
            self.cmd('wte ')  #set tilt-axis to eighth step mode
            time.sleep(2.0)
            self.cmd('r ')    #Reset PTU, this will make PTU turn full range of motion!
        return
    
    def ephem_point(self,ep,imu=None,target='sun',init=True,ptu_cmd=True):
        '''
        Point PTU at sun or moon using ephemeris data and a magnetic North offset from IMU if available
        
        Inputs:
            ep: ephem created with ephem.Observer()
            imu: IMU VN100 connection created with (ie imu=EzAsyncData.connect('COM7', 115200))
            target: target to point at. Current options are 'sun' or 'moon'
            init: (boolean) set to True to set PTU to position mode and speed to 1000 pos/sec
            ptu_cmd: (boolean) set to True to command PTU to point at target (otherwise, will just print az/el angles)
        '''
        self.ep = ep
        self.ep.lat, self.ep.lon, self.ep.elevation = self.lat,self.lon,self.alt
        self.ep.date = datetime.now()
        self.ep.date += ephem.hour*self.utc_off
        
        #Can only point at either sun or moon
        if target == 'sun':
            self.target = ephem.Sun()
        elif target == 'moon':
            self.target = ephem.Moon()
        else:
            print('Ephemeris target not understood, goodbye')
            return
        self.target.compute(self.ep)
        self.target_azel = [self.target.az*180/np.pi,self.target.alt*180/np.pi]
        
        if imu != None:
#            try:
            while (imu.imu.current_data.yaw_pitch_roll is None): #Wait here until there is data
                pass #do nothing
            ypr = imu.imu.current_data.yaw_pitch_roll  #Assuming yaw/pitch/roll data from IMU is in reference to mag-North
            az_off = ypr.x    #store current yaw value of IMU
            self.target_azel[0] += az_off +142
                #imu.imu.disconnect()  #disconnect from IMU
#            except:
#                az_off = float(input('Sun Pointing Error: Could not communicate with IMU\n'+
#                                     'Input azimuth offset in degrees from magnetic North:\n'+
#                                     '>>>'))
#                self.target_azel[0] += az_off       
        else:
            try:
                az_off = float(input('Input azimuth offset in degrees from magnetic North:\n'+
                                     '>>>'))
                self.target_azel[0] += az_off
            except:
                print('Error: could not add manual offset specified')
        
        #Set PTU to position mode and speed to 1000 pos/sec
        if init == True:        
            self.cmd('i ')
            time.sleep(0.1)
            self.cmd('ci ')
            time.sleep(0.5)
            self.cmd('ps1000 ')
            time.sleep(0.1)
            self.cmd('ts1000 ')
            time.sleep(0.1)
        
        print(target+' azimuth =',round(self.target_azel[0],4),'degrees CW from North, '+target+' tilt angle',round(self.target_azel[1],4),'degrees above horizon')
        
        #Command PTU to point at target
        if ptu_cmd==True:
            #Send PTU negative angle if azimuth angle > 180 deg
            if self.target_azel[0]>180:
                pan_off = 'pp' + str(-int((1/self.pan_pdeg)*(360-self.target_azel[0]))) + ' '
            else:
                pan_off = 'pp' + str(int((1/self.pan_pdeg)*self.target_azel[0])) + ' '
            print('Sending PTU command: ',pan_off)
            self.cmd(pan_off)
            time.sleep(0.1)
            tilt_off = 'tp' + str(int((-1/self.tilt_pdeg)*self.target_azel[1])) + ' '
            print('Sending PTU command: ',tilt_off)
            self.cmd(tilt_off)
            time.sleep(0.1)      
                  
if __name__ == '__main__':
    
    #Create a ptu object, define com_port and baudrate
    ptu = PTU(com_port='COM5',baudrate=9600)
    
    #Check PTU step mode in both axes
    print('Step mode pan axis: ',ptu.read('wp '))
    print('Step mode tilt axis: ',ptu.read('wt '))
    print('')
    
    #Set PTU to microstep mode
    ptu.set_microstep()

    
#    Example 1) Point PTU at sun
#               Need ephemeris data to point at sun
    ep = ephem.Observer()
    #Set latitude, longitude and altitude to Blacksburg, VA
    ptu.lat, ptu.lon, ptu.alt = '37.205144','-80.417560', 634
    ptu.utc_off=4   #Set UTC time offset of EST
    
    #Connect to IMU to get offset from magnetic North
    imu = EzAsyncData.connect('COM7', 115200)
    
    ptu.ephem_point(ep,imu=imu,target='sun')
    input('Press any key when PTU is pointed at the sun')
#   
#    
#    #Track the sun for approximately 10 seconds using ephemeris data
#    cnt=0
#    track_time = 10
#    
#    #Make sure ptu is pointed at sun to start
#    ptu.ephem_point(ep,target='sun')
#    input('Press any key when PTU is pointed at the sun')
#    
#    #Begin tracking loop
#    while cnt < track_time:
#        ptu.ephem_point(ep,target='sun',init=False)
#        time.sleep(1)
#        cnt+=1
#    
#    
##    Example 2) Point PTU at moon
##               Need ephemeris data to point at sun   
#    #Point PTU at moon
#    ptu.ephem_point(ep,target='moon')
#    input('Press any key when PTU is pointed at the moon')
