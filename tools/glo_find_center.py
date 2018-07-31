# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 13:12:19 2017

@author: addiewan
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import statsmodels.api as sm
import statsmodels.formula.api as smf
import os
import cv2

user=os.getenv('username')

def glo_find_center(image,gauss_kern=7):
    
    ##threshholds for canny edge detection
    thresh_lo_all = [0,0,0,0,0,0]
    thresh_hi_all = [5,5,5,5,5,5]
    
    ##min/max radius for each channel
    min_rad_all = [60, 90, 86,60,98, 94]
    max_rad_all = [68,115,102,72,139,108]
    
    ## row/col limits for each channel
    row_lim_ch = [(0,500),(0,500),(0, 500),(501,1024),(501,1024),(501,1024)]
    col_lim_ch = [(0,426),(427,852),(853,1280),(0,426),(427,852),(853,1280)]
    
    ##size of hough accumulator for each channel..increase for slower more accurate detection
    accum_all = [10,10,10,10,10,10]    
    
    img = np.uint8(255.0*image/16384.0)
    
    #step 1: blur image using gaussian
    img_g = cv2.GaussianBlur(img,(gauss_kern,gauss_kern),0)
    img_g_ = img_g.copy()
    
    #Step 2: cycle through each channel and locate circle center using hough cirlce algorithm
    #img_g = np.uint8(255.0*img_g/16384.0)
    cen_x = np.zeros(6)
    cen_y = np.zeros(6)
    cen_r = np.zeros(6)
    for ch in np.arange(6):
        circles = cv2.HoughCircles(img_g_,cv2.HOUGH_GRADIENT,1,400,param1=thresh_hi_all[ch],param2=accum_all[ch],minRadius=min_rad_all[ch],maxRadius=max_rad_all[ch])
        if circles is not None:
            for i in range(len(circles[0])):      
             # corresponding to the center of the circle
             x = circles[0][i][0]
             y = circles[0][i][1]
             r = circles[0][i][2]
             if y > row_lim_ch[0][0] and y < row_lim_ch[0][1] and x > col_lim_ch[0][0] and x < col_lim_ch[0][1]:
                 if ch == 0:
                     cen_x[0] = x
                     cen_y[0] = y
                     cen_r[0] = r
             if y > row_lim_ch[1][0] and y < row_lim_ch[1][1] and x > col_lim_ch[1][0] and x < col_lim_ch[1][1]:
                 if ch == 1:
                     cen_x[1] = x
                     cen_y[1] = y
                     cen_r[1] = r                  
             if y > row_lim_ch[2][0] and y < row_lim_ch[2][1] and x > col_lim_ch[2][0] and x < col_lim_ch[2][1]:
                 if ch == 2:
                     cen_x[2] = x
                     cen_y[2] = y
                     cen_r[2] = r 
             if y > row_lim_ch[3][0] and y < row_lim_ch[3][1] and x > col_lim_ch[3][0] and x < col_lim_ch[3][1]:
                 if ch == 3:
                     cen_x[3] = x
                     cen_y[3] = y
                     cen_r[3] = r 
             if y > row_lim_ch[4][0] and y < row_lim_ch[4][1] and x > col_lim_ch[4][0] and x < col_lim_ch[4][1]:
                 if ch == 4:
                     cen_x[4] = x
                     cen_y[4] = y
                     cen_r[4] = r 
             if y > row_lim_ch[5][0] and y < row_lim_ch[5][1] and x > col_lim_ch[5][0] and x < col_lim_ch[5][1]:
                 if ch == 5:
                     cen_x[5] = x
                     cen_y[5] = y
                     cen_r[5] = r 
                     
    return {'cen_x':cen_x,
            'cen_y':cen_y,
            'cen_r':cen_r}
