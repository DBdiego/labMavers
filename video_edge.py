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



vidcap = cv2.VideoCapture('./Videos/zoovideo3.mp4')

while True:
    success,frame = vidcap.read()
    if not success:
        break

    edges = cv2.Canny(frame, 100, 200)

    
    grassbool1 =  frame[:,:,1] < 220
    grassbool2 = frame[:,:,1] > 120
    grassbool3 = frame[:,:,0] < 200
    grassbool4 = frame[:,:,2] < 200
    grassbool5 = frame[:,:,0] > 60
    grassbool6 = frame[:,:,2] > 90
    grass = (grassbool1*grassbool2*grassbool3*grassbool4*grassbool5*grassbool6*255).astype(np.uint8)


    boolarray = edges > 100
    boolgrass = grass > 250
    shift = 50
    zeros_array = np.zeros((shift,640)).astype(int)
    bool_object = boolarray*1
    bool_grass = (grass > 250)*1
    test1_array = np.concatenate(( zeros_array, bool_grass), axis=0)
    test2_array = np.concatenate((bool_object, zeros_array), axis =0)
    grass_over = (test1_array*test2_array*255).astype(np.uint8)
    over_grass = np.delete(grass_over, range(352,352+shift), 0)

    new_edges = (edges - over_grass).astype(np.uint8)
    
    
#    box = 30
#    for i in range(int(255/box)+1):
#        boolgray1 = gray>(255-box*(i+1))
#        boolgray2 = gray<(255-box*i)
#        gray[boolgray1*boolgray2] = 255-box*i if box*i > 0 else 0
#    i_s = []
#    edgebool = boolarray
#    for i in range(len(gray[1,:])):
#        edgelines = edges[:,3*i-2:3*i+2]/255
#        
#        if np.sum(edgelines) >50:
#            i_s.append(2*i-1)
#            print(3*i, np.sum(new_edges))
##    gray[:,i_s] = 255
#    new_gray = gray - clean_edges

    cv2.imshow('frame', new_edges)
    cv2.waitKey(1)
    
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break
    
small = np.zeros((80,80)).astype(np.uint8)
#for i in range(len(small[:,1])): Do this with numpy instead, quicker
#    for j in range(640):
#         som += sum(gray[4*i:4*i+4, j]).astype(np.uint16)
#         if (j+ 1) % 8 == 0 and j != 0:
#             small[i,int((j+1)/8-1)] = som/32
#             som = 0

