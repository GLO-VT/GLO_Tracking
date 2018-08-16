# -*- coding: utf-8 -*-
"""
Created on Thu May 17 14:12:47 2018

@author: Bailey group PC
"""
import minimalmodbus
import time
from datetime import datetime
import pandas as pd
import numpy as np

class SS:
    def __init__(self,inst_id=1,com_port='COM4',baudrate=115200):
        '''
        Class to connect to SolarMEMS ISSDX sun sensor
        
        Inputs:
            inst_id: (int) instrument id (default=1)
            com_port: serial com port for USB to 485-converter
            baudrate: (int) default baudrate = 19200, fastest baudrate = 115200
        '''
        self.inst_id = inst_id
        self.com_port = com_port
        self.baudrate = baudrate
        try:
            self.ss = minimalmodbus.Instrument(self.com_port,self.inst_id)
            self.ss.serial.baudrate=self.baudrate
            self.fov = self.ss.read_register(2)
            print('Connected to sun sensor',self.inst_id,'FOV = +/-',self.fov,'degrees')
        except:
            print('Hey sun sensor',self.inst_id,'why wont you talk to me?') 
            
    def read_data_all(self):
        try:
            self.data = self.ss.read_registers(8,7)    #read SS data registers 8-14
            
            #convert ss data to appropriate units
            self.watts = self.data[1]    #watts/m^2
            self.temp = float(self.data[2])/10.0   #Temperature degrees F
            #ang_x_filt
            if self.data[3] > 50000:
                self.ang_x_filt = -float(65536 - self.data[3])/1000.0
            else:    
                self.ang_x_filt = float(self.data[3])/1000.0
            #ang_y_filt
            if self.data[4] > 50000:
                self.ang_y_filt = -float(65536 - self.data[4])/1000.0
            else:    
                self.ang_y_filt = float(self.data[4])/1000.0
             #ang_x_raw
            if self.data[5] > 50000:
                self.ang_x_raw = -float(65536 - self.data[5])/1000.0
            else:    
                self.ang_x_raw = float(self.data[5])/1000.0
            #ang_y_raw
            if self.data[6] > 50000:
                self.ang_y_raw = -float(65536 - self.data[6])/1000.0
            else:    
                self.ang_y_raw = float(self.data[6])/1000.0 
            #Additional info register - determine if sun in field of view (FOV)
            if self.data[0] == 0:
                self.sun_in_fov = True
            else:
                self.sun_in_fov = False
            #Additional info register set to 255 if radiation <300 W/m^2
            if self.data[0] == 255:
                self.no_sunlight = True
            else:
                self.no_sunlight = False
            self.data_exists=True
        except:
           print('Failed to read data from sun sensor',self.inst_id) 
           self.data_exists=False
           self.watts = np.nan
           self.temp = np.nan
           self.ang_x_raw = np.nan
           self.ang_y_raw = np.nan
           self.ang_x_filt = np.nan
           self.ang_y_filt = np.nan
           
    def read_data_raw(self):
        try:
            self.data = self.ss.read_registers(12,2)    #read SS data registers 8-14
             #ang_x_raw
            if self.data[0] > 50000:
                self.ang_x_raw = -float(65536 - self.data[0])/1000.0
            else:    
                self.ang_x_raw = float(self.data[0])/1000.0
            #ang_y_raw
            if self.data[1] > 50000:
                self.ang_y_raw = -float(65536 - self.data[1])/1000.0
            else:    
                self.ang_y_raw = float(self.data[1])/1000.0 
            self.data_exists=True
        except:
           print('Failed to read data from sun sensor',self.inst_id) 
           self.data_exists=False
           self.ang_x_raw = np.nan
           self.ang_y_raw = np.nan

           
       
            
#if __name__ == '__main__':
    
#    #Example 1) Connect to sun sensors and print data
#    
#    #Connect to three sun sensors (may need to change com port and/or set baudrate to 19200)
#    ss1 = SS(inst_id=1,com_port='COM4',baudrate=115200)
#    ss2 = SS(inst_id=2,com_port='COM4',baudrate=115200)
#    ss3 = SS(inst_id=3,com_port='COM4',baudrate=115200)
           
#    ss1 = SS(inst_id=1,com_port='COM4',baudrate=115200)
#    ss2 = SS(inst_id=2,com_port='COM4',baudrate=115200)
#    ss3 = SS(inst_id=3,com_port='COM4',baudrate=115200)
#    
#    #Read data
#    ss1.read_data_all()
#    ss2.read_data_all()
#    ss3.read_data_all()
#    
#    #Print data
#    print('SS1 ang_x_raw = ', round(ss1.ang_x_raw,3),'##### SS2 ang_x_raw = ', round(ss2.ang_x_raw,3), '##### SS3 ang_x_raw  = ',round(ss3.ang_x_raw,3))
#    print('SS1 ang_y_raw = ', round(ss1.ang_y_raw,3),'##### SS2 ang_y_raw  = ',round(ss2.ang_y_raw,3), '##### SS3 ang_y_raw  = ',round(ss3.ang_y_raw,3))
#    print('SS1 ang_x_filt =',round(ss1.ang_x_filt,3),'##### SS2 ang_x_filt = ',round(ss2.ang_x_filt,3),'##### SS3 ang_x_filt = ',round(ss3.ang_x_filt,3)) 
#    print('SS1 ang_y_filt =',round(ss1.ang_y_filt,3),'##### SS2 ang_y_filt = ',round(ss2.ang_y_filt,3),'##### SS3 ang_y_filt = ',round(ss3.ang_y_filt,3))       
#    print('SS1 watts =     ',     round(ss1.watts,3),'##### SS2 watts = ',     round(ss2.watts,3),     '##### SS3 watts = ',     round(ss3.watts,3))  
#    print('SS1 temp =      ',      round(ss1.temp,3),'##### SS2 temp = ',      round(ss2.temp,3),      '##### SS3 temp = ',      round(ss3.temp,3)) 
#    
#          
#    #Example 2) Collect 5 seconds of data at 10hz and store data in pandas dataframe
#    hz=10
#    rec_time=5
#    delay = 1.0/hz
#    df = pd.DataFrame(columns=['ang_x_filt',
#                               'ang_y_filt',
#                               'ang_x_raw',
#                               'ang_y_raw',
#                               'watts',
#                               'temp',
#                               'elapsed'])
#    
#    #Create a list of all sun sensors (can add/remove sun sensors from this list)
#    ss_all = [ss1,ss2,ss3]
#    
#    #Create a dictionary to store data in
#    data = {}
#    for i in range(len(ss_all)):
#        data['ss'+str(ss_all[i].inst_id)] = df
#    
    #Collect and store data until rec_time expires
#    t0=time.time()
#    while time.time() - t0 < rec_time:
#        d_time=datetime.now()
#        for i in range(len(ss_all)):
#            
#            #Read data
#            ss_all[i].read_data_all()
#            
#            #Store data in list
#            data_add = [ss_all[i].ang_x_filt,
#                        ss_all[i].ang_y_filt,
#                        ss_all[i].ang_x_raw,
#                        ss_all[i].ang_y_raw,
#                        ss_all[i].watts,                         
#                        ss_all[i].temp,
#                        time.time() - t0]
#            
#            #Add list as a row into the dataframe
#            data['ss'+str(ss_all[i].inst_id)].loc[d_time] = data_add
#            
#        #Ensure that this loop iterates at desired data rate   
#        t_diff = time.time() - t0
#        if (delay - t_diff) > 0:
#            time.sleep(delay - t_diff)
#        else:
#            time.sleep(delay)
            
#    hz=10
#    rec_time=50000
#    delay = 1.0/hz            
#    t0=time.time()
#    while time.time() - t0 < rec_time:
#        d_time=datetime.now()
#            
#        #Read data
#        ss2.read_data_all()
#        
#        print('x=',ss2.ang_x_raw,'y=',ss2.ang_y_raw)
#            
#        #Ensure that this loop iterates at desired data rate   
#        t_diff = time.time() - t0
#        if (delay - t_diff) > 0:
#            time.sleep(delay - t_diff)
#        else:
#            time.sleep(delay)            
#        