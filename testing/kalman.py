# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 21:53:29 2018

@author: addiewan
"""
import numpy as np
from numpy.random import randn

def predict(u,P,F,Q):
    u=np.dot(F,u)
    P=np.dot(F,np.dot(P,F.T)) + Q
    
    return u,P

def correct(u,A,b,P,Q,R):
    C=np.dot(A,np.dot(P,A.T)) + R
    K=np.dot(P,np.dot(A.T,np.linalg.inv(C)))
    
    u=u+np.dot(K,(b-np.dot(A,u)))
    P=P-np.dot(K,np.dot(C,K.T))
    
    return u,P

hz=20
dt=1/hz
A=np.array([[1,0],
            [0,1]])
    
u=np.array([[ 0.],
            [ 0.]])
    
#random initial measurement centered at state value
b=np.array([[u[0,0] + randn(1)[0]],
            [u[1,0] + randn(1)[0]]])
    
#P: covariance matrix
P=np.array([[ 0.01,  0.  ],
            [ 0.  ,  0.01]])
    
#Prediction matrix (model)
F=np.array([[1.0,dt],
            [0.0,1.0]])
    
#Unit variance for the sake of simplicity
#uncertainty from environment
Q=np.array([[ 1.,  0.],
            [ 0.,  1.]])
    
R=np.array([[ 1.,  0.],
            [ 0.,  1.]])

N=100
predictions,corrections,measurements = [],[],[]
for k in np.arange(0,N):
    u,P=predict(u,P,F,Q)
    predictions.append(u)
    u,P=correct(u,A,b,P,Q,R)
    corrections.append(u)
    measurements.append(b)
    b=np.array([[u[0,0] + randn(1)[0]],
                 [u[1,0] + randn(1)[0]]])

print('predicted final estimate:',predictions[-1][0])
print('corrected final estimates:',corrections[-1][0])
print('measured state:',measurements[-1][0])
