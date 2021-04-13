# pip install opencv-python
# pip install numpy
# pip install msgpack

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

    image_width = 512
    image_border = 100
    image_high = image_width - image_border
    x = image_width / 2
    y = image_width / 2
    # angle = 20 # degrees
    tan_angle = 0.3639702342662023613510478827768340438904717837537381141956129887

    # Formulas for x,y calculation depending on z
    #
    # tg(angle) = half_width / height
    # half_width = tg(angle) * height
    #
    # half_width -> 256 pixels
    # ? -> dx, dy (pixels)
    # dx (m) = (dx_pixels * half_width) / 256
    #
    # 256 / dx => a
    # half_width / a = dx(m)
    # (half_width * dx) / 256

    position = np.array([0, 0, 0], dtype = np.float32)
    dx = 0
    dy = 0

    old_image = []
    old_points = np.array([[x,y]], dtype = np.float32)

    # selects image center
    def select_point(): 
        global point, old_points, x ,y
        x = image_width / 2
        y = image_width / 2
        point = (x,y)
        old_points = np.array([[x,y]], dtype = np.float32)

    def simulationStepStarted(msg):
        a = 5
        
    def simulationStepDone(msg):
        client.doNextStep=True

    # Handles new data from bottom proximity sensor        
    def bottomProximitySensorCallback(msg):
        if msg[0] == False:
            return
        detected = msg[1]
        z = msg[2]
        if detected != 0:
            position[2] = z
        else:
            print('Bottom proximity sensor out of range')
    
    # Handles new image from vision sensor
    def imageCallback(msg):
        global x, y, old_image, old_points
        resolution = msg[1]
        image = bytearray(msg[2])
        sensorImage = np.array(image, dtype=np.uint8)
        sensorImage.resize([resolution[1],resolution[0],3])
        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_BGR2GRAY)
        (_, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)

        if (x > image_high) or (x < image_border) or (y > image_high) or (y < image_border):
            select_point()

        if (old_image == []):
            old_image = sensorImage.copy()
        new_points, _, _ = cv2.calcOpticalFlowPyrLK(old_image, sensorImage, old_points, None, **lk_params)
        old_image = sensorImage.copy() #current frame becomes previous
        dx = new_points[0][0] - old_points[0][0]
        dy = new_points[0][1] - old_points[0][1]
        old_points = new_points #current x,y points become previous

        half_width = tan_angle * position[2]
        position[0] += (dx * half_width) / (image_width / 2)
        position[1] += (dy * half_width) / (image_width / 2)
        # position[2] is set by bottomProximitySensorCallback()
        print([round(num, 2) for num in position])

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
    bottomProximitySensorHandle=client.simxGetObjectHandle('bottom_proximity_sensor',client.simxServiceCall())

    cv2.destroyAllWindows()
    cv2.namedWindow('frame')

    if client.runInSynchronousMode:
        client.simxSynchronous(True)

    client.simxGetVisionSensorImage(visionSensorHandle[1],False,client.simxDefaultSubscriber(imageCallback))
    client.simxReadProximitySensor(bottomProximitySensorHandle[1],client.simxDefaultSubscriber(bottomProximitySensorCallback))

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