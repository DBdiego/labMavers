import numpy as np
import cv2
import os

path = './Videos/Nikhil_the_best_auto'
all_files = [path + '/' + f for f in os.listdir(path)]
frames = [cv2.imread(image) for image in all_files]
print(len(frames))
#beam_angle = 60
#ground = np.zeros((100,100))
#for i in range(50):
#    ground[i,i:50] = 1
#    ground [50+i, i:100] = 1


#grass = np.nonzero(ground)    

#print(grass)


