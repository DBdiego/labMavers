import glob
import numpy as np
import cv2
import pdb
from scipy import signal
import time


# Hyper parameters
kernel_value = 30       ### Square that is averaged out for less noise
number_areas = 5       ### Number of vertical areas created for the centroids
delay        = 0.1     ### delay in seconds for better observation of algorithm performance
num_pixels_above = 10  ### batch of pixels to be white above the centroid (per step of 10 in this case)

#Bools
show_ground   = 1   # Show comparison of original video with green edges and colour filter with direction proposal
show_edges    = 0   # Show edges (Black and White)
show_off_mode = 1   # If turned of, none of the images is shown. This is to evaluate performance


# Parameters regarding the indicator drawings (for the red cross and the blue dots on show_ground comparison)
# advised not to change these
step   = 12  
gap    =  5
lw     =  1
gap_in =  3


# Importing and reading video
vidcap = cv2.VideoCapture('./Videos/zoovideo6.mp4')

# Creating kernel for averageing pixel windows
kernel = np.ones((kernel_value, kernel_value),np.uint8)

# Font for percentages on show_ground comparison image
font = cv2.FONT_HERSHEY_SIMPLEX


# Looping through frames of the video
while True:
    success, frame = vidcap.read()
    if not success:
        print('Video Finished')
        break

    # Adding delay
    time.sleep(delay)

    # Starting performance clock
    start_time = time.time()

    # Converting RGB to YUV since Bebob works in YUV
    yuv_orig = cv2.cvtColor(frame, cv2.COLOR_RGB2YUV)
    yuv = cv2.morphologyEx(yuv_orig, cv2.MORPH_CLOSE, kernel)
    #yuv = yuv_orig
    
    # Color Filer (YUV)
    # --> y-value check
    b1 = yuv[:,:,0] > 70  #int(0.30 * 255)
    b2 = yuv[:,:,0] < 240 #int(0.95 * 255)

    # -->u-value check
    b3 = yuv[:,:,1] < 140
    
    # -->v-value check
    b4 = yuv[:,:,2] < 110

    grass = (b1 * b2 * b3 * b4 * 255).astype(np.uint8)


    # Edge detection (only needed when displaying image)
    if show_off_mode:
        edges = cv2.Canny(grass, 100, 200)

    
    img_height, img_width = np.shape(grass)
    new_RGB = cv2.cvtColor(grass, cv2.COLOR_GRAY2RGB)

    cy_data = []
    cx_data = []
    percentages = []
    area_width = int(img_width/number_areas)

    # Computing Centroids of white areas in different sub-areas of the frame
    for i in range(number_areas):
        v_lines_x = area_width * (i+1)
        new_RGB[:, v_lines_x-1:v_lines_x+1] = [255, 0, 0]

        local_grass_area = grass[:, i*area_width:(i+1)*area_width]

        # Percentage that tis "green" compared to total area
        perc = (np.sum(local_grass_area)/255)/(area_width * img_height)*100
        perc_text = str(round(perc, 1))+'%'

        # Moment of inertia of area
        M = cv2.moments(local_grass_area)
        if M["m00"]:
            cx = int(M["m10"] / M["m00"]) + v_lines_x - area_width
            cy = int(M["m01"] / M["m00"])
        else:
            cx = 0
            cy = 0

        # Append data for decision purposes later
        cx_data.append(cx)
        cy_data.append(cy)
        percentages.append(perc)

        if show_off_mode:
            # Drawing computations on the image (not important in real life)
            new_RGB[cy-gap_in:cy+gap_in, cx-gap_in:cx+gap_in] = [255, 0, 0]
    
        cv2.putText(new_RGB, perc_text, (int(v_lines_x-3*area_width/4),100), font, 0.5,(255,255,255),2,cv2.LINE_AA)

    # Defining the goals
    goal_index = np.argmax(np.array(percentages)*np.array(cy_data))
    x_goal = cx_data[goal_index]

    y_column = grass[:,x_goal][::-1]


    # Determining y-position of dirction
    for i in range(len(y_column)-cy_data[goal_index], len(y_column), num_pixels_above):
        if np.sum(y_column[i:i+num_pixels_above])/255 == num_pixels_above:
            coordinate = len(y_column) - (i+num_pixels_above)
            
            # Check for y-position not to bee too far from centroid (max 0.5 of a frame size)
            if (cy_data[goal_index] - coordinate)/len(y_column) <= 0.3:
                y_goal = len(y_column) - (i+num_pixels_above)


    # Drawing indicator of choice (not important in real life)
    if show_off_mode:
        color = [0, 0, 255]
        new_RGB[y_goal-step-gap:y_goal-gap     , x_goal-lw:x_goal+lw] = color
        new_RGB[y_goal+gap     :y_goal+step+gap, x_goal-lw:x_goal+lw] = color
        
        new_RGB[y_goal-lw:y_goal+lw , x_goal-gap-step:x_goal-gap     ] = color
        new_RGB[y_goal-lw:y_goal+lw , x_goal+gap     :x_goal+gap+step] = color

        

    #print(time.time() - start_time)
    original_RGB = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)

    if show_off_mode:
        
        # Adding edges on both original and processed frame
        frame  [edges>0] = [0, 0, 255]
        new_RGB[edges>0] = [0, 0, 255]
        
        #creatig a comparison frame (gluing both original and processed)
        comparison = np.hstack((frame, new_RGB))
        
        if show_ground:
            cv2.imshow('frame', comparison)
            
        elif show_edges:
            cv2.imshow('frame', edges)
        
    cv2.waitKey(1)
    
    
    
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()











'''
OLD CODE SNIPPETS
'''

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
