# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 21:53:29 2018

"""
import numpy as np
import matplotlib.pyplot as plt
from grab_data import grab_data
import os
import pandas as pd
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
#####################################

data_loc = cwd + '/laptop_tracking_20180619/'
params_loc = cwd + '/20180619_Sun_Sensor_laptop_runs.txt'
data,data_all_ss1,data_all_ss2,data_all_ss3,data_all = grab_data(data_loc,params_loc)

#Pick a sun sensor run to use for the "Kalman Filter"
run=4
mask = data_all_ss1['run'] == run
time = elapsed=data_all_ss1.loc[mask,'elapsed']-data_all_ss1.loc[mask,'elapsed'][0]
ss_track_x = data_all_ss1.loc[mask,'ang_x_track']
##pre-calculate velocities (in real-time code this will run with latest three samples)
#vel=np.zeros(len(ss_track_x))
#vel[0:2]=0.0
#for i in range(2,len(ss_track_x)):
    
    
imu_ang_z = data_all_ss1.loc[mask,'imu_ang_z']
ptu_cmd_x = data_all_ss1.loc[mask,'ptu_cmd_x']

#def ddx(x,y):
#    return (np.array([2/((x[1]-x[0])*(x[2]-x[0])),
#                    -2/((x[2]-x[1])*(x[1]-x[0])),
#                    2/((x[2]-x[1])*(x[2]-x[0]))])*y.T).sum()

#Kalman Predictor stage
def predict(x,A,u,B,P,Q):
    x=np.dot(A,x) + np.dot(B,u)
    P=np.dot(A,np.dot(P,A.T)) + Q  
    return x,P

#Kalman Correction stage
def correct(x,H,z,P,Q,R):
    C=np.dot(H,np.dot(P,H.T)) + R
    K=np.dot(P,np.dot(H.T,np.linalg.inv(C)))
    #print(K)
    
    x=x+np.dot(K,(z-np.dot(H,x)))
    P=P-np.dot(K,np.dot(H,K.T))
    
    return x,P
dt=time.diff().mean()
hz=1/dt
ptu_step2deg = 23.1428/3600


#Initial state estimate 
x=np.array([[0.],  #sun_pos_x
            [0.],  #ptu_vel_x
            [0.],  #gon_vel_x
            [0.]]) #sun_acc_x

#Prediction matrix (model)
A=np.array([[ 1.0 , dt  ,  dt  , 0.5*dt**2 ],
            [ 0.0 , 0.0 , -1.0 ,-0.5*dt**2 ],
            [ 0.0 , 0.0 ,  1.0 , 0.5*dt**2 ],
            [ 0.0 , 0.0 ,  0.0 , 1.0       ]])
    
#P: covariance matrix (covariance between state variables)
P=np.array([[0.01, 0.  ,  0.  , 0.  ],
            [0.  , 0.01,  0.  , 0.  ],
            [0.  , 0.  ,0.01  , 0.  ],
            [0.  , 0.  ,0.    , 0.01]])
#P=np.cov(np.array([vel+ptu_cmd_x*ptu_step2deg,pos]))
    
#uncertainty from environment
Q=np.array([[0.01, 0.  ,  0.  , 0.  ],
            [0.  , 0.01,  0.  , 0.  ],
            [0.  , 0.  ,0.01  , 0.  ],
            [0.  , 0.  ,0.    , 0.01]])

#Initial control commands 
#u=np.array([[ptu_cmd_dx[2]*ptu_step2deg]])
u=np.array([[0.0]])
    
#Control Matrix    
B=np.array([[dt],
            [1.0],
            [0.0],
            [0.0]])   
    
#B=np.array([[0.0,0.0,0.0],
#            [0.0,0.0,0.0],
#            [0.0,0.0,0.0]]) 

#Initial sensor measurements
z=np.array([[ss_track_x[0]],
            [ss_track_x[1]],
            [ss_track_x[2]],
            [imu_ang_z[2]]])
    
#Sensor matrix: just identity matrix because sensors measure the state
#H=np.array([[0.0,0.0,1.0],
#            [0.5*dt,0.0,0.5*dt]
#            [2./((x[1]-x[0])*(x[2]-x[0])),-2./((x[2]-x[1])*(x[1]-x[0])),2./((x[2]-x[1])*(x[2]-x[0]))]])

#H=np.array([[0.0             ,0.0            ,1.0             ],
#            [-1./(2*dt)       ,0.0           ,1./(2*dt)       ],
#            [2./((dt)*(2*dt)),-2./((dt)*(dt)),2./((dt)*(2*dt))]])
#    
#H=np.array([[1.0  ,-2*dt ,-0.5*(2*dt)**2 ],
#            [1.0  ,-dt   ,-0.5*(dt)**2   ],
#            [1.0  ,0.0   ,0.0            ]])
    
H=np.array([[ 1.0  , 2*dt  , 2*dt , 2*(dt)**2],
            [ 1.0  , dt    , dt   , 2*(dt)**2],
            [ 1.0  , 0.0   , 0.0  , 0.       ],
            [ 0.0  , 0.0   , 1.0  , 0.       ]])
    
#Unit variance for the sake of simplicity

#Uncertainty from sensors 
R=np.array([[ 0.015, 0.   , 0.    , 0.0],
            [ 0.   , 0.015, 0.    , 0.0],
            [ 0.   , 0.   , 0.015 , 0.0],
            [ 0.   , 0.   , 0.    , 0.1]])

N=len(time)
predictions,corrections,measurements = [],[],[]
#pred=pd.DataFrame(columns=['pos','vel','acc'])
#corr=pd.DataFrame(columns=['pos','vel','acc'])
#meas=pd.DataFrame(columns=['pos','vel','acc'])
data=pd.DataFrame(columns=['pred_sun_pos','corr_sun_pos','meas_sun_pos',
                           'pred_ptu_vel','corr_ptu_vel','meas_ptu_vel',
                           'pred_gon_vel','corr_gon_vel','meas_gon_vel',
                           'pred_sun_acc','corr_sun_acc','meas_sun_acc',
                           'elapsed'])
for k in np.arange(2,N-1):
    #Update sensor measurements
    z=np.array([[ss_track_x[k-2]],
                [ss_track_x[k-1]],
                [ss_track_x[k]  ],
                [imu_ang_z[k]   ]])
    #Update ptu control commands
    u=np.array([[ptu_cmd_x[k]*ptu_step2deg]]) 
    u=np.array([[0.0]]) 
    
    x,P=predict(x,A,u,B,P,Q)
    data.loc[k,['pred_sun_pos','pred_ptu_vel','pred_gon_vel','pred_sun_acc']]=[x[0][0],x[1][0],x[2][0],x[3][0]]
    predictions.append(x)
    x,P=correct(x,H,z,P,Q,R)
    data.loc[k,['corr_sun_pos','corr_ptu_vel','corr_gon_vel','corr_sun_acc']]=[x[0][0],x[1][0],x[2][0],x[3][0]]
    corrections.append(x)
    measurements.append(z)
#    data.loc[k,['meas_pos','meas_vel','meas_acc']]=[z[0][0],z[1][0],z[2][0]]
    data.loc[k,['meas_sun_pos']]=z[2][0]
    #data.loc[k,['meas_ptu_vel']]=[z[0][0]*(-1./(2*dt)) + z[2][0]*(1./(2*dt))]
    data.loc[k,['meas_sun_acc']]=[(z[0][0]*2./((dt)*(2*dt))) - 
                                  (z[1][0]*2./((dt)*(dt))) +
                                  (z[2][0]*2./((dt)*(2*dt)))]
    data.loc[k,['elapsed']]=time[k]

print('predicted final estimate:',predictions[-1][0])
print('corrected final estimates:',corrections[-1][0])
print('measured state:',measurements[-1][0])

pos_measure=np.zeros(N-1)
vel_measure=np.zeros(N-1)
pos_predict=np.zeros(N-1)
vel_predict=np.zeros(N-1)
pos_correct=np.zeros(N-1)
vel_correct=np.zeros(N-1)
for i in range(len(measurements)):
    pos_measure[i]=measurements[i][0][0]
    vel_measure[i]=measurements[i][1][0]
    pos_predict[i]=predictions[i][0][0]
    vel_predict[i]=predictions[i][1][0]
    pos_correct[i]=corrections[i][0][0]
    vel_correct[i]=corrections[i][1][0]

plt.figure()
plt.plot(data.index,data.pred_sun_pos,'-o',label='predicted position')
plt.plot(data.index,data.corr_sun_pos,'-o',label='corrected position')
plt.plot(data.index,data.meas_sun_pos,'-o',label='measured position')
plt.plot(data.index,ss_track_x[3:],'-o',label='ss_track_x')
plt.legend()

plt.figure()
plt.plot(data.index,data.pred_vel,label='predicted velocity')
plt.plot(data.index,data.corr_vel,label='corrected velocity')
plt.plot(data.index,data.meas_vel,label='measured velocity')
plt.plot(data.index,ptu_cmd_x[3:]*ptu_step2deg,label='ptu_cmd_x')
plt.plot(data.index,imu_ang_z[3:],label='imu_ang_z')
plt.plot(data.index,imu_ang_z[3:]+ptu_cmd_x[3:]*ptu_step2deg,label='imu_ang_x+ptu_cmd_x')
plt.legend()

plt.figure()
plt.plot(data.index,data.meas_pos-data.corr_pos,'-o',label='position error')
plt.plot(data.index,data.meas_vel-data.corr_vel,'-o',label='velocity error')
plt.legend()
    
#plt.figure()
#plt.plot(pos_measure,label='measured')
#plt.plot(pos_predict,label='predicted')
#plt.plot(pos_correct,label='corrected')
#plt.title('Position offset')
#plt.legend()
#
#plt.figure()
#plt.plot(vel_measure,label='measured')
#plt.plot(vel_predict,label='predicted')
#plt.plot(vel_correct,label='corrected')
#plt.title('Velocity offset')
#plt.legend()
#    
#plt.figure()
#plt.plot(pos_correct-pos_measure,label='Position (predicted - measured) ')
##plt.plot(vel_correct-vel_measure,label='Velocity (predicted - measured) ')
#plt.title('Corrected vs. Measured')
#plt.legend()
#
#plt.figure()
#plt.plot(vel_correct-vel_measure,label='Velocity (predicted - measured) ')
##plt.plot(vel_correct-vel_measure,label='Velocity (predicted - measured) ')
#plt.title('Corrected vs. Measured')
#plt.legend()