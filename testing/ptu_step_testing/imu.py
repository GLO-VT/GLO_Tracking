# -*- coding: utf-8 -*-
"""
Created on Thu May 17 14:12:47 2018

@author: Bailey group PC
"""

from vnpy import EzAsyncData

class IMU:
    '''
    Class to connect to VectorNav VN100 IMU
    
    Inputs:
        com_port: serial com port for USB to 485-converter
        baudrate: (int) default baudrate = 19200, fastest baudrate = 115200
    '''
    def __init__(self,com_port='COM5',baudrate=115200):
        self.com_port = com_port
        self.baudrate = baudrate
        try:
            self.imu = EzAsyncData.connect(self.com_port,self.baudrate)
        except:
            print('Communication with IMU failed')
            return
        
    def grab_accel(self): 
        '''
        Return current IMU acceleration 
        '''
        while (self.imu.current_data.acceleration is None): #Wait here until there is data
            pass #do nothing
        self.accel = self.imu.current_data.acceleration
        return self.accel
        
    def grab_ang_r(self): 
        '''
        Return current IMU angular rates (radian/sec)
        '''
        while (self.imu.current_data.angular_rate is None): #Wait here until there is data
            pass #do nothing
        self.ang_r = self.imu.current_data.angular_rate
        return self.ang_r
    
    def grab_mag(self):
        '''
        Return current IMU magnetic field vector
        '''
        while (self.imu.current_data.magnetic is None): #Wait here until there is data
            pass #do nothing
        self.mag = self.imu.current_data.magnetic
        return self.mag
        
    def grab_ypr(self):
        '''
        Return current IMU yaw/pitch/roll
        '''
        while (self.imu.current_data.yaw_pitch_roll is None): #Wait here until there is data
            pass #do nothing
        self.ypr = self.imu.current_data.yaw_pitch_roll
        return self.ypr
    
    def change_baudrate(self,baudrate):
        '''
        change baudrate of imu
        '''
        self.baudrate = baudrate
        self.imu.sensor.change_baudrate(self.baudrate)
        self.imu.sensor.disconnect()
        try:
            self.imu = EzAsyncData.connect(self.com_port,self.baudrate)
        except:
            print('Tried to change IMU baudrate to',self.baudrate,'...could not reconnect to IMU')
            return
                  
#if __name__ == '__main__':
#    
#    #Create an imu object, define com_port and baudrate
#    imu = IMU(com_port='COM7',baudrate=115200) #921600
#    
#    imu.grab_accel()
#    print('Accel-x =',imu.accel.x)
#    print('Accel-y =',imu.accel.y)
#    print('Accel-z =',imu.accel.z)
#    
#    imu.grab_ang_r()
#    print('Angular Rate x-axis =',imu.ang_r.x)
#    print('Angular Rate y-axis =',imu.ang_r.y)
#    print('Angular Rate z-axis =',imu.ang_r.z)
#    
#    imu.grab_mag()
#    print('Magnetic Field Vector x =',imu.mag.x)
#    print('Magnetic Field Vector y =',imu.mag.y)
#    print('Magnetic Field Vector z =',imu.mag.z)
#    
#    
#    #Disconnect from imu
#    imu.imu.disconnect()
#    
#    #Create an imu object, define com_port and baudrate
#    imu = IMU(com_port='COM7',baudrate=115200) #921600
#    imu.change_baudrate(921600)
#    imu.imu.disconnect()
#    imu = IMU(com_port='COM7',baudrate=921600)    
#    N=1000
#    imu_ang_z = np.zeros(N)
#    for i in range(N):
            
        
