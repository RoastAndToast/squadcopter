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

    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    direction = FORWARD

    sensorValues = [0 for i in range(8)]


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

    # def forwardProximitySensorCallback(msg):
    def directionProximitySensorCallback(sensor_id, msg):
        if msg[0] == False:
            return
        detected = msg[1]
        if detected != 0:
            sensorValues[sensor_id] = msg[2]
        else:
            sensorValues[sensor_id] = -1 # -1 == infinity

    def F_ProximitySensorCallback(msg):
        sensor_id = 0
        directionProximitySensorCallback(sensor_id,msg)
    def FR_ProximitySensorCallback(msg):
        sensor_id = 1
        directionProximitySensorCallback(sensor_id,msg)
    def R_ProximitySensorCallback(msg):
        sensor_id = 2
        directionProximitySensorCallback(sensor_id,msg)
    def BR_ProximitySensorCallback(msg):
        sensor_id = 3
        directionProximitySensorCallback(sensor_id,msg)
    def B_ProximitySensorCallback(msg):
        sensor_id = 4
        directionProximitySensorCallback(sensor_id,msg)
    def BL_ProximitySensorCallback(msg):
        sensor_id = 5
        directionProximitySensorCallback(sensor_id,msg)
    def L_ProximitySensorCallback(msg):
        sensor_id = 6
        directionProximitySensorCallback(sensor_id,msg)
    def FL_ProximitySensorCallback(msg):
        sensor_id = 7
        directionProximitySensorCallback(sensor_id,msg)
        
        
    def setDirection(dx, dy):
        if (abs(dx) > abs(dy)):
            if (dx > 0):
                direction = RIGHT
            else:
                direction = LEFT
        else:
            if (dy > 0):
                direction = FORWARD
            else: 
                direction = BACKWARD
        print(direction)

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
        sensorImage = cv2.flip(sensorImage,0)
        (_, sensorImage) = cv2.threshold(sensorImage, 200, 255, cv2.THRESH_TRUNC)

        if (x > image_high) or (x < image_border) or (y > image_high) or (y < image_border):
            select_point()

        #if (old_image == None): ??
        if (old_image == []):
            old_image = sensorImage.copy()
        new_points, _, _ = cv2.calcOpticalFlowPyrLK(old_image, sensorImage, old_points, None, **lk_params)
        old_image = sensorImage.copy() #current frame becomes previous
        dy = new_points[0][0] - old_points[0][0] # 13.04.2021_2 Maksims Terjohins fix - swapped dx and dy for propper output
        dx = new_points[0][1] - old_points[0][1]
        setDirection(dx,dy)
        old_points = new_points #current x,y points become previous

        half_width = tan_angle * position[2]
        position[0] += (dx * half_width) / (image_width / 2)
        position[1] += (dy * half_width) / (image_width / 2)
        # position[2] is set by bottomProximitySensorCallback()
        # print([round(num, 2) for num in position])

        x,y = new_points.ravel()

        displayedImage = cv2.cvtColor(sensorImage, cv2.COLOR_GRAY2BGR)
        cv2.circle(displayedImage, (x,y),8, (0,0,255),-1) # draw red circle
        cv2.imshow('frame',displayedImage)
        
    def stepSimulation():
        if client.runInSynchronousMode:
            while not client.doNextStep:
                client.simxSpinOnce()
            client.doNextStep=False
            client.simxSynchronousTrigger()
        else:
            client.simxSpinOnce()

    visionSensorHandle=client.simxGetObjectHandle('Vision_sensor',client.simxServiceCall())
    quadcopterTargetHandle=client.simxGetObjectHandle('Quadcopter_target',client.simxServiceCall())
    # quadcopterHandle = client.simxGetObjectHandle('Quadcopter_base', client.simxServiceCall())
    bottomProximitySensorHandle=client.simxGetObjectHandle('bottom_proximity_sensor',client.simxServiceCall())

    # bottomProximitySensorHandle=client.simxGetObjectHandle('bottom_proximity_sensor',client.simxServiceCall())
    # clockwise, 0 for forward sensor

    # sensor handle array
    sensorHandles=[]
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_f', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_fr', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_r', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_br', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_b', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_bl', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_l', client.simxServiceCall()))
    sensorHandles.append(client.simxGetObjectHandle('Proximity_sensor_fl', client.simxServiceCall()))


    cv2.destroyAllWindows()
    cv2.namedWindow('frame')

    if client.runInSynchronousMode:
        client.simxSynchronous(True)

    client.simxGetVisionSensorImage(visionSensorHandle[1],False,client.simxDefaultSubscriber(imageCallback))
    client.simxReadProximitySensor(bottomProximitySensorHandle[1],client.simxDefaultSubscriber(bottomProximitySensorCallback))
    #client.simxReadProximitySensor(sensorHandles[0][1],client.simxDefaultSubscriber(forwardProximitySensorCallback))
    
    # 8 subscriptions for each sensor
    client.simxReadProximitySensor(sensorHandles[0][1],client.simxDefaultSubscriber(F_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[1][1],client.simxDefaultSubscriber(FR_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[2][1],client.simxDefaultSubscriber(R_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[3][1],client.simxDefaultSubscriber(BR_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[4][1],client.simxDefaultSubscriber(B_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[5][1],client.simxDefaultSubscriber(BL_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[6][1],client.simxDefaultSubscriber(L_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[0][1],client.simxDefaultSubscriber(FL_ProximitySensorCallback))


    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    client.simxStartSimulation(client.simxDefaultPublisher())

    quadcopterTargetInitMsg = client.simxGetObjectPosition(quadcopterTargetHandle[1], -1, client.simxServiceCall())
    quadcopterTargetPos = quadcopterTargetInitMsg[1]
    
    temp = 0
    # main loop
    while True:
        stepSimulation()
        # drone control example:
        if temp < 100:
            quadcopterTargetPos[1] += 0.005
            # client.simxSetObjectPosition(quadcopterTargetHandle[1], -1, quadcopterTargetPos, client.simxServiceCall())
            temp += 1
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
        
    client.simxStopSimulation(client.simxDefaultPublisher())