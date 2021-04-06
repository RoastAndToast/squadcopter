import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cv2 import cv2
import array
import sys
import numpy as np
import math
import time

# pip install matplotlib
# pip install opencv-python

# code style - both camelCase and snake_case are used

# Optimisation:
# Initial time per 100 images processed: ~19.0 seconds
# Contrary to my expectations, removing resizing doesn't give any effect.
# Changing cv2.waitKey(30) to cv2.waitKey(1) gives a profit of ~3.0 seconds per 100 operations (16.0 instead of 19.0)
# Btw, removing video output gives one more second of profit. When we are sure algorithm works fine, consider removing this output.
# 
# time to read:  15.37683969999999
# time to select point:  0.002240399999983822
# time to calculate:  0.10556910000001807
# time to print:  0.8240265999999821
# 
# reading image timings:
# get image time:  11.891000700000014
# array time:  3.2803406999999822
# resize time:  0.0007657000000094172
# filter time:  0.017509200000013436
# threshold time:  0.003592699999984461

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

#point_selected = False
#old_points = np.array([[]])   

# Lucas Kanade parameters
lk_params = dict(winSize = (50,50),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

x = 255
y = 255

def select_point(): 
    global point, old_points, x ,y
    x = 255
    y = 255
    point = (x,y)
    old_points = np.array([[x,y]], dtype = np.float32)


def streamVisionSensor():
    global point_selected, old_points
    # Mouse function
    '''def select_point_mouse(event,x,y, flags, params): 
        global point, point_selected, old_points
        if event == cv2.EVENT_LBUTTONDOWN:
            point = (x,y)
            point_selected = True
            old_points = np.array([[x,y]], dtype = np.float32)

    def select_point(): 
        global point, old_points
        point = (255,255)
        x = point[0]
        y = point[1]
        old_points = np.array([[x,y]], dtype = np.float32)
    '''
    cv2.destroyAllWindows()
    cv2.namedWindow('frame')

    errorCode,visionSensorHandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
    errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_streaming)
    time.sleep(0.5)
    # print('here')

    global x, y

    # errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_buffer)
    errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_streaming)
    # print(resolution)
    # print(image)
    sensorImage = np.array(image,dtype=np.uint8)
    print(sensorImage)
    sensorImage.resize([resolution[1],resolution[0],3])
    sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
    sensorImage = cv2.flip(sensorImage,0)
    (thresh, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)
    old_image = sensorImage.copy()
    select_point()

    start_time = time.perf_counter()
    tick_counter = 0

    timer_checkpoint = start_time
    timer_read_image = 0
    timer_calc = 0
    timer_print = 0
    timer_select_point = 0

    timer_get_image = 0
    timer_array = 0
    timer_resize = 0
    timer_filter = 0
    timer_threshold = 0

    # while (sim.simxGetConnectionId(clientID)!=-1): # try changing to while True: - less secure, but must be faster
    while True:
        # errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_buffer) # if we don't use error futher, maybe python will let us not to read it?
        timer_checkpoint = time.perf_counter()
        # _,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_buffer)
        _,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_streaming)
        timer_checkpoint_temp = time.perf_counter()
        timer_get_image += (timer_checkpoint_temp - timer_checkpoint)
        timer_checkpoint = timer_checkpoint_temp

        sensorImage = np.array(image,dtype=np.uint8)
        timer_checkpoint_temp = time.perf_counter()
        timer_array += (timer_checkpoint_temp - timer_checkpoint)
        timer_checkpoint = timer_checkpoint_temp

        sensorImage.resize([resolution[1],resolution[0],3])
        timer_checkpoint_temp = time.perf_counter()
        timer_resize += (timer_checkpoint_temp - timer_checkpoint)
        timer_checkpoint = timer_checkpoint_temp

        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
        timer_checkpoint_temp = time.perf_counter()
        timer_filter += (timer_checkpoint_temp - timer_checkpoint)
        timer_checkpoint = timer_checkpoint_temp

        sensorImage = cv2.flip(sensorImage,0) # why do wee need it? it's a complicated operation
        # (thresh, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC) # if we don't use thresh futher, maybe python will let us not to read it?
        (_, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)
        timer_checkpoint_temp = time.perf_counter()
        timer_threshold += (timer_checkpoint_temp - timer_checkpoint)
        timer_checkpoint = timer_checkpoint_temp

        # timer_read_image += (time.perf_counter() - timer_checkpoint)
        # timer_checkpoint = time.perf_counter()
        
        if (x > 512.0) or (x<0.0) or (y>512.0) or (y<0.0):
            select_point() # set point to image center

        # timer_select_point += (time.perf_counter() - timer_checkpoint)

        # timer_checkpoint = time.perf_counter()

        # new_points, status, error = cv2.calcOpticalFlowPyrLK(old_image, sensorImage, old_points, None, **lk_params) # solve warning # ignore status, error?
        new_points, _, _ = cv2.calcOpticalFlowPyrLK(old_image, sensorImage, old_points, None, **lk_params)
        old_image = sensorImage.copy() #current frame becomes previous
        old_points = new_points #current x,y points become previous
        x,y = new_points.ravel()

        # timer_calc += (time.perf_counter() - timer_checkpoint)

        # timer_checkpoint = time.perf_counter()
            
        # actually we only need to display image for debug. so when we are sure it works well, we can remove image displaying.
        # if we do need to show it, then i don't think wee have to resize it agagin and apply a filter.

        #displayedImage = cv2.resize(sensorImage, (480,480)) # image that we see at the output
        #displayedImage = cv2.cvtColor(displayedImage, cv2.COLOR_GRAY2BGR) # back to color from grayscale
        #cv2.circle(displayedImage, (x,y),8, (0,0,255),-1) # draw red circle
        #cv2.imshow('frame',displayedImage) # show the output

        # is async possible to show image?

        cv2.circle(sensorImage, (x,y),8, (0,0,255),-1) # draw red circle
        cv2.imshow('frame',sensorImage) # show the output

        
        #if cv2.waitKey(30) & 0xFF == ord('q'): # press q to pay respect # cv2.waitKey(30) - check delay time
            #break
        if cv2.waitKey(1) & 0xFF == ord('q'): # press q to pay respect # cv2.waitKey(30) - check delay time
            break

        # timer_print += (time.perf_counter() - timer_checkpoint)

        tick_counter += 1
        # print(tick_counter)
        if tick_counter >= 100:
            # print('time: ', time.perf_counter() - start_time)
            # print('time to read: ', timer_read_image)
            # print('time to select point: ', timer_select_point)
            # print('time to calculate: ', timer_calc)
            # print('time to print: ', timer_print)
            timer_read_image = 0
            timer_select_point = 0
            timer_calc = 0
            timer_print = 0

            print('get image time: ', timer_get_image)
            print('array time: ', timer_array)
            print('resize time: ', timer_resize)
            print('filter time: ', timer_filter)
            print('threshold time: ', timer_threshold)

            timer_get_image = 0
            timer_array = 0
            timer_resize = 0
            timer_filter = 0
            timer_threshold = 0

            start_time = time.perf_counter()
            tick_counter = 0
         
    cv2.destroyAllWindows()    
    print ('End of Simulation')
    point_selected = False


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
select_point() # image center
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(0.5)

    point = ()
    old_points = np.array([[]])    
    point_selected = True

    
    streamVisionSensor()
    

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for exaplte):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')



