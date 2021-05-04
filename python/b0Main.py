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
    
    # drone current direction
    direction = FORWARD

    # is drone in finish?
    isFinish = False

    # drone movement speed
    speed = 0.005

    sensorValues = [0 for i in range(8)]

    # returns rotated direction by 90 degrees to the left
    def turnLeft():
        if direction == FORWARD:
            return LEFT
        elif direction == RIGHT:
            return FORWARD
        elif direction == BACKWARD:
            return RIGHT
        else:
            return BACKWARD

    # returns rotated direction by 90 degrees to the right
    def turnRight():
        if direction == FORWARD:
            return RIGHT
        elif direction == RIGHT:
            return BACKWARD
        elif direction == BACKWARD:
            return LEFT
        else:
            return FORWARD

    # returns rotated direction by 180 degrees
    def turnBackward():
        if direction == FORWARD:
            return BACKWARD
        elif direction == RIGHT:
            return LEFT
        elif direction == BACKWARD:
            return FORWARD
        else:
            return RIGHT

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

    # returns true, if drone is on crossroad or turn
    def isCrossroadOrTurn(sensors):
        left = (sensors[0] < 0)
        right = (sensors[4] < 0)
        return left or right

    # returns true, if drone is in crossroad.
    # at least two pathes (except backwards) should exist
    # based on sensor values given from getForwardDistances()
    def isCrossRoad(sensors):
        left = (sensors[0] < 0)
        forward = (sensors[2] < 0)
        right = (sensors[4] < 0)
        return (left and forward) or (left and right) or (forward and right)

    # returns true, if drone is exaclty in the middle of crossroad.
    # based on diagonal sensor values
    def isCrossroadMiddle():
        # print(sensorValues[1], sensorValues[3], sensorValues[5], sensorValues[7])
        return ((abs(sensorValues[1] - sensorValues[3]) <= 0.1)
        and (abs(sensorValues[1] - sensorValues[5]) <= 0.1)
        and (abs(sensorValues[1] - sensorValues[7]) <= 0.1))

    # get 5 sensor values (l, fl, f, fr, r) according to movement direction
    def getForwardDistances():
        if direction == FORWARD:
            return sensorValues[6:8] + sensorValues[0:3]
        elif direction == RIGHT:
            return sensorValues[0:5]
        elif direction == BACKWARD:
            return sensorValues[2:7]
        else:
            return sensorValues[4:8] + sensorValues[0:1]

    # checks if drone is in finish
    def finishCheck(msg):
        global isFinish
        if msg[0] == False:
            return
        isFinish = msg[1]

    # Direction proximity sensor callback
    def directionProximitySensorCallback(sensor_id, msg):
        if msg[0] == False:
            return
        detected = msg[1]
        if detected != 0:
            sensorValues[sensor_id] = msg[2]
        else:
            sensorValues[sensor_id] = -1 # -1 == infinity

    # Callbacks for side proximity sensors:
    def F_ProximitySensorCallback(msg):
        directionProximitySensorCallback(0,msg)
    def FR_ProximitySensorCallback(msg):
        directionProximitySensorCallback(1,msg)
    def R_ProximitySensorCallback(msg):
        directionProximitySensorCallback(2,msg)
    def BR_ProximitySensorCallback(msg):
        directionProximitySensorCallback(3,msg)
    def B_ProximitySensorCallback(msg):
        directionProximitySensorCallback(4,msg)
    def BL_ProximitySensorCallback(msg):
        directionProximitySensorCallback(5,msg)
    def L_ProximitySensorCallback(msg):
        directionProximitySensorCallback(6,msg)
    def FL_ProximitySensorCallback(msg):
        directionProximitySensorCallback(7,msg)
        
    # Set direction value
    def setDirection(dx, dy):
        global direction
        if (abs(dx) > abs(dy)):
            if (dx > 0):
                direction = FORWARD
            else:
                direction = BACKWARD
        else:
            if (dy > 0):
                direction = LEFT
            else: 
                direction = RIGHT

    # print direction
    def printDirection():
        if direction == FORWARD:
            print('forward')
        elif direction == BACKWARD:
            print('backward')
        elif direction == LEFT:
            print('left')
        else:
            print('right')

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
        dx = new_points[0][0] - old_points[0][0] # 13.04.2021_2 Maksims Terjohins fix - swapped dx and dy for propper output
        dy = new_points[0][1] - old_points[0][1]
        # setDirection(dx,dy)
        old_points = new_points #current x,y points become previous

        half_width = tan_angle * position[2]
        position[0] += (dx * half_width) / (image_width / 2)
        position[1] += (dy * half_width) / (image_width / 2)
        # position[2] is set by bottomProximitySensorCallback()
        print([round(num, 2) for num in position])

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
    quadcopterBaseHandle = client.simxGetObjectHandle('Quadcopter_base', client.simxServiceCall())
    quadcopterHandle = client.simxGetObjectHandle('Quadcopter', client.simxServiceCall())
    bottomProximitySensorHandle=client.simxGetObjectHandle('bottom_proximity_sensor',client.simxServiceCall())
    finishFirstHandle = client.simxGetObjectHandle('Finish_first', client.simxServiceCall())

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
    
    # 8 subscriptions for each sensor
    client.simxReadProximitySensor(sensorHandles[0][1],client.simxDefaultSubscriber(F_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[1][1],client.simxDefaultSubscriber(FR_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[2][1],client.simxDefaultSubscriber(R_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[3][1],client.simxDefaultSubscriber(BR_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[4][1],client.simxDefaultSubscriber(B_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[5][1],client.simxDefaultSubscriber(BL_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[6][1],client.simxDefaultSubscriber(L_ProximitySensorCallback))
    client.simxReadProximitySensor(sensorHandles[7][1],client.simxDefaultSubscriber(FL_ProximitySensorCallback))

    client.simxCheckCollision(quadcopterHandle[1], finishFirstHandle[1], client.simxDefaultSubscriber(finishCheck))


    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    client.simxStartSimulation(client.simxDefaultPublisher())

    quadcopterTargetInitMsg = client.simxGetObjectPosition(quadcopterTargetHandle[1], -1, client.simxServiceCall())
    quadcopterTargetPos = quadcopterTargetInitMsg[1]
    
    crossRoadProceeded = False
    # main loop
    while not isFinish:
        stepSimulation()
        forwardDistances = getForwardDistances()

        # ir strupcels
        if (not crossRoadProceeded and (forwardDistances[0] > 0 and forwardDistances[2] > 0 and forwardDistances[4] > 0)):
            # print('blocker')
            direction = turnBackward()
            crossRoadProceeded = True
        
        # if crossroad or turn
        elif (not crossRoadProceeded and isCrossroadOrTurn(forwardDistances)):
            # print('crossroad or turn')
            
            # we can't know is it crossroad or turn till it's middle
            if isCrossroadMiddle():
                # print('middle')
                
                # if crossroad
                if isCrossRoad(forwardDistances):
                    print('is crossroad')
                
                # if turn
                elif ((forwardDistances[0] < 0) or (forwardDistances[4] < 0)):
                    # print('turn')
                    if (forwardDistances[0] < 0):
                        direction = turnLeft()
                    elif (forwardDistances[4] < 0):
                        direction = turnRight()
                    crossRoadProceeded = True
        
        # if decision has already been made on this crossroad/turn
        elif not ((forwardDistances[0] < 0) or (forwardDistances[4] < 0)):
            crossRoadProceeded = False

        # movement
        if (direction == FORWARD) or (direction == BACKWARD):
            coord = 1
        else:
            coord = 0
        if (direction == BACKWARD) or (direction == LEFT):
            multiplier = -1
        else:
            multiplier = 1
        quadcopterTargetPos[coord] += multiplier * speed
        client.simxSetObjectPosition(quadcopterTargetHandle[1], -1, quadcopterTargetPos, client.simxServiceCall())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
        
    # client.simxStopSimulation(client.simxDefaultPublisher())
