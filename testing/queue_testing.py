# -*- coding: utf-8 -*-
"""
Created on Wed Jun 27 11:04:59 2018
@author: Bailey group PC
"""


'''
name = process name
target = name of function you are running
args: parameters needed to pass to function
'''
import multiprocessing
import time


DataFetch = multiprocessing.Process(name='DataFetch',target=ss_read,args=(params.delay,
                                                                                 params.track_time,
                                                                                 params.filter_mode,
                                                                                 params.ss_read,
                                                                                 params.SS1_id,
                                                                                 params.SS1_com,
                                                                                 params.SS1_baudrate,
                                                                                 params.SS2_id,
                                                                                 params.SS2_com,
                                                                                 params.SS2_baudrate,
                                                                                 params.SS3_id,
                                                                                 params.SS3_com,
                                                                                 params.SS3_baudrate))



def sunsensor_read(delay,
            track_time,
            filter_mode,
            ss_read,
            SS1_id,
            SS1_com,
            SS1_baudrate,
            SS2_id,
            SS2_com,
            SS2_baudrate,
            SS3_id,
            SS3_com,
            SS3_baudrate):
    
    ss_read=[1,2,3]

    ss1=SS(inst_id=SS1_id,com_port=SS1_com,baudrate=SS1_baudrate)
    ss2=SS(inst_id=SS2_id,com_port=SS2_com,baudrate=SS2_baudrate)
    ss3=SS(inst_id=SS3_id,com_port=SS3_com,baudrate=SS3_baudrate)
    
    time_start = time.time()
    while True:
        #SS1
        time_ss1_0=time.time()
        ss1.read_data_all() #Read all data from sun sensor using SS class      
        time_ss1_1=time.time()
        ss1_tstamp=(time_ss1_1-time_ss1_0)/2.
        if filter_mode == 2:   #Use filtered data if filter mode=2, otherwise use raw data
            if(SS1_x_offset_queue.full()==False):
                SS1_x_offset_queue.put(ss1.ang_x_filt + ss_eshim_x[0])   #add electronic shims to angle offset for tracking
            if(SS1_y_offset_queue.full()==False):
                SS1_y_offset_queue.put(ss1.ang_y_filt + ss_eshim_y[0])
            if(SS1_time_offset_queue.full()==False):
                SS1_time_offset_queue.put(ss1_tstamp)  
        else:
            if(SS1_x_offset_queue.full()==False):
                SS1_x_offset_queue.put(ss1.ang_x_raw + ss_eshim_x[0])   #add electronic shims to angle offset for tracking
            if(SS1_y_offset_queue.full()==False):
                SS1_y_offset_queue.put(ss1.ang_y_raw + ss_eshim_y[0])
            if(SS1_time_offset_queue.full()==False):
                SS1_time_offset_queue.put(ss1_tstamp)
        
        #SS2
        time_ss2_0=time.time()
        ss2.read_data_all() #Read all data from sun sensor using SS class      
        time_ss2_1=time.time()
        ss2_tstamp=(time_ss2_1-time_ss2_0)/2.
        if filter_mode == 2:   #Use filtered data if filter mode=2, otherwise use raw data
            if(SS2_x_offset_queue.full()==False):
                SS2_x_offset_queue.put(ss2.ang_x_filt + ss_eshim_x[1])   #add electronic shims to angle offset for tracking
            if(SS2_y_offset_queue.full()==False):
                SS2_y_offset_queue.put(ss2.ang_y_filt + ss_eshim_y[1])
            if(SS2_time_offset_queue.full()==False):
                SS2_time_offset_queue.put(ss2_tstamp)  
        else:
            if(SS2_x_offset_queue.full()==False):
                SS2_x_offset_queue.put(ss2.ang_x_raw + ss_eshim_x[1])   #add electronic shims to angle offset for tracking
            if(SS2_y_offset_queue.full()==False):
                SS2_y_offset_queue.put(ss2.ang_y_raw + ss_eshim_y[1])
            if(SS2_time_offset_queue.full()==False):
                SS2_time_offset_queue.put(ss2_tstamp)
            
        #SS3
        time_ss3_0=time.time()
        ss3.read_data_all() #Read all data from sun sensor using SS class      
        time_ss3_1=time.time()
        ss3_tstamp=(time_ss3_1-time_ss3_0)/2.
        if filter_mode == 2:   #Use filtered data if filter mode=2, otherwise use raw data
            if(SS3_x_offset_queue.full()==False):
                SS3_x_offset_queue.put(ss3.ang_x_filt + ss_eshim_x[2])   #add electronic shims to angle offset for tracking
            if(SS3_y_offset_queue.full()==False):
                SS3_y_offset_queue.put(ss3.ang_y_filt + ss_eshim_y[2])
            if(SS3_time_offset_queue.full()==False):
                SS3_time_offset_queue.put(ss3_tstamp)  
        else:
            if(SS3_x_offset_queue.full()==False):
                SS3_x_offset_queue.put(ss3.ang_x_raw + ss_eshim_x[2])   #add electronic shims to angle offset for tracking
            if(SS3_y_offset_queue.full()==False):
                SS3_y_offset_queue.put(ss3.ang_y_raw + ss_eshim_y[2])
            if(SS3_time_offset_queue.full()==False):
                SS3_time_offset_queue.put(ss3_tstamp)
        #Ensure you are at the rate    
        time_diff = time.time() - time_ss1_0
        if delay - time_diff > 0:
            time.sleep(delay-time_diff)
        # Exit the function once tracking time has been reached
        if((time.time()-time_start) > track_time):
            #Note: try catch to stop PTU is not commented here, ensure if it would be able to be called
            #ptu.cmd('ps0')
            #ptu.cmd('ts0')
            print("Tracking complete!")
            return
