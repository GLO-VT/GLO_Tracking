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
ptu_sim = multiprocessing.Process(name='ptu_simulate',target=ptu_simulate,args=(params.delay,
                                                                                 params.track_time,
                                                                                 params.ss_read,
                                                                                 params.ptu48_baudrate,
                                                                                 ptu_cmd_list,
                                                                                 params.save_loc+'run_'+str(run_num),
                                                                                 file_prefix,
                                                                                 params.ptu48_kp,
                                                                                 params.ptu48_ks,
                                                                                 params.ptu48_ka,
                                                                                 params.ptu48_vd,
                                                                               params.ptu48_pd))

ss_read=[1,2,3]



ss1=SS(inst_id=1,com_port='COM6',baudrate=115200)
ss2=SS(inst_id=2,com_port='COM4',baudrate=115200)
ss3=SS(inst_id=3,com_port='COM8',baudrate=115200)



    #Loop through all sun sensors
time_ss1_0=time.time()
ss1.read_data_all()    #Read all data from sun sensor using SS class      
time_ss1_1=time.time()
ss1_tstamp=time_ss1_1 - (time_ss1_1-time_ss1_0)/2.
SS1_x_offset.queue = ss1.ang_x_filt + ss_eshim_x[0]    #add electronic shims to angle offset for tracking  

  
if filter_mode == 2:   #Use filtered data if filter mode=2, otherwise use raw data
        ang_x[i-1] = ss[i-1].ang_x_filt + ss_eshim_x[i-1]    #add electronic shims to angle offset for tracking
        ang_y[i-1] = ss[i-1].ang_y_filt + ss_eshim_y[i-1]
    else:
        ang_x[i-1] = ss[i-1].ang_x_raw + ss_eshim_x[i-1]
        ang_y[i-1] = ss[i-1].ang_y_raw + ss_eshim_y[i-1]