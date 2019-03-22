import csv
import numpy as np

#this file reads the csv states and link them with the frames
path = 'simulation/log/' 
name = '20190322-104528.csv' #should be changed after each measurement/simulation

# read time, x, y, vx, vy, theta, psi, q, r  from csv
with open(path+name, 'rt') as f:
    reader = csv.reader(f)
    states = (np.array(list(reader))[1:,[0,1,2,4,5,8,9,11,12]]).astype(float)


def get_states_from_im(image_name):
    image_float = float(image_name.split('.jpg')[0]) # remove .jpg and make float
    search = True
    index = 0

    if image_float > states[-1,0]:  # if the frame is later than the last measurement, use last measurement
        index = len(states)-1

    elif image_float < states[0,0]: # if the frame is before 1st measurement, use 1st
        index = 0

    else:
        while search: # find which index corresponds with the frame (often no iteration is used)
            index = int( round((image_float-states[index,0]) / (states[index+1,0]-states[index,0])) ) + index

            if abs(states[index, 0]-image_float) < abs(states[index+1, 0]-image_float) and abs(states[index, 0]-image_float) < abs(states[index-1, 0]-image_float):
               search = False
               
    return(index)


test_file_name = '13.123.jpg'
print(get_states_from_im(test_file_name)) # index

index = get_states_from_im(test_file_name)
print(states[index,0]) # time
print(states[index,1]) # x
print(states[index,2]) # y
print(states[index,3]) # vx
print(states[index,4]) # vy
print(states[index,5]) # theta
print(states[index,6]) # psi
print(states[index,7]) # q
print(states[index,8]) # r
