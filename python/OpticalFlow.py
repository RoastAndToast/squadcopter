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
    cv2.namedWindow('frame')

    errorCode,visionSensorHandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
    errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_streaming)
    time.sleep(0.5)


    errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_buffer)
    sensorImage = np.array(image,dtype=np.uint8)
    sensorImage.resize([resolution[1],resolution[0],3])
    sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
    sensorImage = cv2.flip(sensorImage,0)
    (thresh, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)
    old_image = sensorImage.copy()
    select_point()

    while (sim.simxGetConnectionId(clientID)!=-1): 
        errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,visionSensorHandle,0,sim.simx_opmode_buffer)
        sensorImage = np.array(image,dtype=np.uint8)
        sensorImage.resize([resolution[1],resolution[0],3])
        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
        sensorImage = cv2.flip(sensorImage,0)
        (thresh, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)
        global x,y

        if (x > 512.0) or (x<0.0) or (y>512.0) or (y<0.0):
            select_point() # set point to image center

        new_points, status, error = cv2.calcOpticalFlowPyrLK(old_image, sensorImage, old_points, None, **lk_params)
        old_image = sensorImage.copy() #current frame becomes previous
        old_points = new_points #current x,y points become previous
        x,y = new_points.ravel()               
            
        displayedImage = cv2.resize(sensorImage, (480,480)) # image that we see at the output
        displayedImage = cv2.cvtColor(displayedImage, cv2.COLOR_GRAY2BGR) # back to color from grayscale
        cv2.circle(displayedImage, (x,y),8, (0,0,255),-1) # draw red circle
        cv2.imshow('frame',displayedImage) # show the output
        
        if cv2.waitKey(30) & 0xFF == ord('q'): # press q to pay respect
            break
         
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



