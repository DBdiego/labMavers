import glob
import numpy as np
import cv2
import pdb
from scipy import signal
import time
import os
import glob, os

# Read all jpg files from path directory HARDCODED sorted
path = './Videos/Nikhil_the_best_auto'
files42 = [ f for f in sorted(glob.glob(os.path.join( path + '/' + '*.jpg'))) if len(f) == 42]
files43 = [ f for f in sorted(glob.glob(os.path.join( path + '/' + '*.jpg'))) if len(f) == 43]
all_files = files42 + files43



frames = [cv2.imread(image) for image in all_files]
#print (all_files)



for frame in frames:
    # Adding delay (could also be done by changing waitKey(1) to a bigger number)
    time.sleep(0.04)
    cv2.imshow('frame', frame)    
    cv2.waitKey(1)
    
    
    
#    if cv2.waitKey(1) and 0xFF == ord('q'):
#        break
    
cv2.destroyAllWindows()

