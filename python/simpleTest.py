# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import numpy as np
import math

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

import time

def retrieveImage():
    #Get the handle of vision sensor
    global errorCode 
    errorCode,visionSensorHandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)

    fig = plt.figure()
    sensorImage = readDataFromVisionSensor(visionSensorHandle)
    global img
    img = plt.imshow(sensorImage,origin='lower', animated = True)
    #plt.imshow(sensorImage,origin='lower')
    ani = animation.FuncAnimation(fig, updateFig(sensorImage), interval=30, blit=True)

    plt.show()

def readDataFromVisionSensor(Handle):
    #Get the image of vision sensor
    errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,Handle,0,sim.simx_opmode_streaming)
    time.sleep(0.1)
    errorCode,resolution,image = sim.simxGetVisionSensorImage(clientID,Handle,0,sim.simx_opmode_buffer)
    sensorImage = []
    sensorImage = np.array(image,dtype = np.uint8)
    sensorImage.resize([resolution[0],resolution[1],3])
    return sensorImage

def updateFig(sensorImage):
    img.set_array(sensorImage)

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    while(True):
        retrieveImage()
        time.sleep(0.03)
        #if 0xFF == ord('q'): # delay between frames 30ms, stop if 'q' pressed
        #    break 
    

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for exaplte):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')



