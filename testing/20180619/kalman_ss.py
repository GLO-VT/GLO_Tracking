# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 21:53:29 2018

"""
import numpy as np
import matplotlib.pyplot as plt
from grab_data import grab_data
import os
cwd = os.getcwd()

data_loc = cwd + '/laptop_tracking_20180619/'
params_loc = cwd + '/20180619_Sun_Sensor_laptop_runs.txt'
data,data_all_ss1,data_all_ss2,data_all_ss3,data_all = grab_data(data_loc,params_loc)

#Pick a sun sensor run to use for the "Kalman Filter"
run=10
mask = data_all_ss1['run'] == run
time = elapsed=data_all_ss1.loc[mask,'elapsed']-data_all_ss1.loc[mask,'elapsed'][0]
pos = data_all_ss1.loc[mask,'ang_x_track']
vel = data_all_ss1.loc[mask,'imu_ang_z']
ptu_cmd_x = data_all_ss1.loc[mask,'ptu_cmd_x']

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

hz=20
dt=1/hz
ptu_step2deg = 23.1428/3600
A=np.array([[1,0],
            [0,1]])

#Initial state estimate 
#x=np.array([[pos[0]],
#            [vel[0]+ptu_cmd_x[0]*ptu_step2deg]])
x=np.array([[0.],
            [0.]])

#Prediction matrix (model)
A=np.array([[1.0,dt],
            [0.0,1.0]])
    
#P: covariance matrix (covariance between state variables)
#P=np.array([[ 0.01,  0.  ],
#            [ 0.  ,  0.01]])
P=np.cov(np.array([vel+ptu_cmd_x*ptu_step2deg,pos]))
    
#uncertainty from environment
Q=np.array([[ .01,  0.],
            [ 0.,  0.01]])

#Initial control commands 
u=np.array([[0.0],
            [ptu_cmd_x[0]*ptu_step2deg]])
    
#Control Matrix    
B=np.array([[0.0,dt],
            [0.0,0.0]])    

#Initial sensor measurements
z=np.array([[pos[0]],
            [vel[0]+ptu_cmd_x[0]*ptu_step2deg]])
    
#Sensor matrix: just identity matrix because sensors measure the state
H=np.array([[1.0,0.0],
            [0.0,1.0]])
    
#Unit variance for the sake of simplicity

#Uncertainty from sensors 
R=np.array([[ 0.015,  0.],
            [ 0.,  0.01]])

N=len(time)
predictions,corrections,measurements = [],[],[]
for k in np.arange(0,N-1):
    x,P=predict(x,A,u,B,P,Q)
    predictions.append(x)
    x,P=correct(x,H,z,P,Q,R)
    corrections.append(x)
    measurements.append(z)
    #Update sensor measurements
    z=np.array([[pos[k+1]],
                [vel[k+1]+ptu_cmd_x[k+1]*ptu_step2deg]])
    #Update ptu control commands
    u=np.array([[0.0],
                [ptu_cmd_x[k+1]*ptu_step2deg]]) 

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
plt.plot(pos_measure,label='measured')
plt.plot(pos_predict,label='predicted')
plt.plot(pos_correct,label='corrected')
plt.title('Position offset')
plt.legend()

plt.figure()
plt.plot(vel_measure,label='measured')
plt.plot(vel_predict,label='predicted')
plt.plot(vel_correct,label='corrected')
plt.title('Velocity offset')
plt.legend()
    
plt.figure()
plt.plot(pos_predict-pos_measure,label='predicted - measured')
plt.title('Velocity offset')
plt.legend()