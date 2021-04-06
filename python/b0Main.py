# pip install opencv-python

import b0RemoteApi
import time
from cv2 import cv2
import numpy as np

with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:
    client.doNextStep=True
    client.runInSynchronousMode=True

    # Lucas Kanade parameters
    lk_params = dict(winSize = (50,50),
                    maxLevel = 2,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    x = 255
    y = 255
    old_image = []
    old_points = np.array([[x,y]], dtype = np.float32)

    # selects image center
    def select_point(): 
        global point, old_points, x ,y
        x = 255
        y = 255
        point = (x,y)
        old_points = np.array([[x,y]], dtype = np.float32)

    def simulationStepStarted(msg):
        a = 5
        
    def simulationStepDone(msg):
        client.doNextStep=True
        
    # Handles new image from vision sensor
    def imageCallback(msg):
        global x, y, old_image, old_points
        resolution = msg[1]
        image = bytearray(msg[2])
        sensorImage = np.array(image, dtype=np.uint8)
        sensorImage.resize([resolution[1],resolution[0],3])
        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
        (_, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)

        if (x > 512.0) or (x<0.0) or (y>512.0) or (y<0.0):
            select_point()

        if (old_image == []):
            old_image = sensorImage.copy()
        new_points, _, _ = cv2.calcOpticalFlowPyrLK(old_image, sensorImage, old_points, None, **lk_params)
        old_image = sensorImage.copy() #current frame becomes previous
        old_points = new_points #current x,y points become previous
        x,y = new_points.ravel()

        cv2.circle(sensorImage, (x,y),8, (0,0,255),-1) # draw red circle
        cv2.imshow('frame',sensorImage)
        
    def stepSimulation():
        if client.runInSynchronousMode:
            while not client.doNextStep:
                client.simxSpinOnce()
            client.doNextStep=False
            client.simxSynchronousTrigger()
        else:
            client.simxSpinOnce()

    visionSensorHandle=client.simxGetObjectHandle('Vision_sensor',client.simxServiceCall())

    cv2.destroyAllWindows()
    cv2.namedWindow('frame')

    if client.runInSynchronousMode:
        client.simxSynchronous(True)

    client.simxGetVisionSensorImage(visionSensorHandle[1],False,client.simxDefaultSubscriber(imageCallback))

    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    client.simxStartSimulation(client.simxDefaultPublisher())
    
    # main loop
    while True:
        stepSimulation()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
        
    client.simxStopSimulation(client.simxDefaultPublisher())