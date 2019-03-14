# -*- coding: utf-8 -*-
"""
Created on Sat Mar  9 09:35:47 2019

@author: Stavrow
"""
import glob
import numpy as np
import cv2
import pdb
from scipy import signal
import time



kernel_value = 20
number_areas = 4
delay = 0.0

font = cv2.FONT_HERSHEY_SIMPLEX


kernel = np.ones((kernel_value, kernel_value),np.uint8)

vidcap = cv2.VideoCapture('./Videos/zoovideo3.mp4')

show_ground = 1
show_edges  = 0



while True:
    success, frame = vidcap.read()
    if not success:
        break

    time.sleep(delay)

    start_time = time.time()
    
    yuv_orig = cv2.cvtColor(frame, cv2.COLOR_RGB2YUV)
    yuv = cv2.morphologyEx(yuv_orig, cv2.MORPH_CLOSE, kernel)

    # y-value check
    b1 = yuv[:,:,0] > 70  #int(0.30 * 255)
    b2 = yuv[:,:,0] < 200 #int(0.95 * 255)

    # u-value check
    b3 = yuv[:,:,1] < 140
    
    # v-value check
    b4 = yuv[:,:,2] < 110

    grass = (b1 * b2 * b3 * b4 * 255).astype(np.uint8)
    edges = cv2.Canny(grass, 100, 200)

    
    img_height, img_width = np.shape(grass)


    y, x = np.nonzero(grass)
    x_goal = x[np.argmin(y)]
    y_goal = y[np.argmin(y)]

    
    step  = 12
    gap   = 5
    lw    = 1

    gap_in = 3

    new_RGB = cv2.cvtColor(grass, cv2.COLOR_GRAY2RGB)

    c_data = []
    percentages = []
    area_width = int(img_width/number_areas)
    

    
    for i in range(number_areas):
        v_lines_x = area_width * (i+1)
        new_RGB[:, v_lines_x-1:v_lines_x+1] = [255, 0, 0]

        local_grass_area = grass[:, i*area_width:(i+1)*area_width]
        
        perc = np.sum(local_grass_area)/(img_width * img_height)
        perc_text = str(round(perc, 1))+'%'
        M = cv2.moments(local_grass_area)
        if M["m00"]:
            cx = int(M["m10"] / M["m00"]) + v_lines_x - area_width
            cy = int(M["m01"] / M["m00"])
        else:
            cx = 0
            cy = 0

        c_data.append([cx, cy])
        percentages.append(perc)

        new_RGB[cy-gap_in:cy+gap_in, cx-gap_in:cx+gap_in] = [255, 0, 0]

        cv2.putText(new_RGB, perc_text, (int(v_lines_x-3*area_width/4),100), font, 0.5,(255,255,255),2,cv2.LINE_AA)

    c_goal = c_data[np.argmax(percentages)]
    x_goal = c_goal[0]
    y_goal = c_goal[1]
    
    color = [0, 0, 255]
    new_RGB[y_goal-step-gap:y_goal-gap     , x_goal-lw:x_goal+lw] = color
    new_RGB[y_goal+gap     :y_goal+step+gap, x_goal-lw:x_goal+lw] = color
    
    new_RGB[y_goal-lw:y_goal+lw , x_goal-gap-step:x_goal-gap     ] = color
    new_RGB[y_goal-lw:y_goal+lw , x_goal+gap     :x_goal+gap+step] = color

        

    '''

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # I think there is a method to read the image directly in gray but this works for now
    # gray_less_noise = signal.convolve2d(gray, anti_noise, boundary='symm', mode='same').astype(np.uint8) # applying the ones filter (from deep learning course) could be used instead if frame
    edges = cv2.Canny(frame, 100, 200) # magic filter, Maybe changing 100, and 200 I dont know what they are yet

    yuv = cv2.morphologyEx(yuv, cv2.MORPH_CLOSE, kernel)

    #Grass detection, if want to make it faster, comment this out till new_edges = ..., and use cv2.imshow('frame', edges) Grass only works for zoovideo3
    #print(yuv[245:255, 315:325])
    # y-value check
    grassbool1 = yuv[:,:,0] > 75  #int(0.30 * 255)
    grassbool2 = yuv[:,:,0] < 200 #int(0.95 * 255)

    # u-value check
    grassbool3 = yuv[:,:,1] < 140
    
    # v-value check
    grassbool4 = yuv[:,:,2] < 110
    '''
    #grassbool3 = yuv[:,:,0] < 200
    #grassbool4 = yuv[:,:,2] < 200
    #grassbool5 = yuv[:,:,0] > 60
    #grassbool6 = yuv[:,:,2] > 90
    '''
    grassbool1 = frame[:,:,1] < 220
    grassbool2 = frame[:,:,1] > 120
    grassbool3 = frame[:,:,0] < 200
    grassbool4 = frame[:,:,2] < 200
    grassbool5 = frame[:,:,0] > 60
    grassbool6 = frame[:,:,2] > 90
    '''
    # grass = (grassbool1*grassbool2*grassbool3*grassbool4*255).astype(np.uint8) #*grassbool3*grassbool4*grassbool5*grassbool6*255).astype(np.uint8)
    # grass = (grassbool1*grassbool2*255).astype(np.uint8)

    '''
    boolarray = edges > 100
    boolgrass = grass > 250
    shift = 50
    zeros_array = np.zeros((shift,640)).astype(int)
    bool_object = boolarray*1
    bool_grass  = (grass > 250)*1
    test1_array = np.concatenate(( zeros_array, bool_grass), axis=0)
    test2_array = np.concatenate((bool_object, zeros_array), axis=0)
    grass_over = (test1_array*test2_array*255).astype(np.uint8)
    over_grass = np.delete(grass_over, range(352,352+shift), 0)
    new_edges = (edges - over_grass).astype(np.uint8)
    '''
    print(time.time() - start_time)
    original_RGB = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)
    
    frame[edges>0] = [0, 0, 255]
    new_RGB[edges>0] = [0, 0, 255]
    
    #cv2.imshow('frame', new_edges)
    #grass[245:255, 315:325] = 100
    #grass[250, 370] = 0

    comparison = np.hstack((frame, new_RGB))
    #cv2.imshow('frame', grass)
    if show_ground:
        cv2.imshow('frame', comparison)
        
    elif show_edges:
        cv2.imshow('frame', edges)
        
    cv2.waitKey(1)
    
    
    
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()
