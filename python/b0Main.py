# pip install opencv-python
# pip install numpy
# pip install msgpack

import b0RemoteApi
import time
from cv2 import cv2
import numpy as np

# Crossroad class.
#
# Stores crossroad coordinates
class Crossroad:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Mark class.
#
# Stores crossroad index, direction from crossroad and it's mark. It can be 0, 1 or 2
class Mark:
    def __init__(self, crossroad, direction, mark):
        self.crossroad = crossroad
        self.direction = direction
        self.mark = mark

with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:
    client.doNextStep=True
    client.runInSynchronousMode=True

    # list of Crossroad
    crossroads = []

    # list of Mark
    marks = []

    # Lucas Kanade parameters
    lk_params = dict(winSize = (30,30),
                    maxLevel = 2,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    image_width = 128
    image_border = 35
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

    correctionCoef = 0.15
    threshold = 0.05
    FORWARD = 0
    BACKWARD = 1
    LEFT = 2
    RIGHT = 3
    
    # drone current direction
    direction = FORWARD

    # is drone in finish?
    isFinish = False

    # drone movement speed
    speed = 0.01

    sensorValues = [0 for i in range(8)]

    # add Crossroad object to crossroads list
    def setVisitedCrossroad(x, y):
        crossroads.append(Crossroad(x, y))
        return len(crossroads) - 1

    # get index of crossroad (x, y) in crossroads list.
    # if no such crossroad in list, returns -1
    def getVisitedCrossroadIndex(x, y):
        eps = 0.1
        for i in range(len(crossroads)):
            crossroad = crossroads[i]
            if (abs(x - crossroad.x) < eps) and (abs(y - crossroad.y) < eps):
                return i
        return -1
    
    # get mark of crossroad and direction.
    # if no such crossroad and direction in marks list, creates such with a mark of 0.
    def getDirectionMark(crossroad, direction):
        for i in range(len(marks)):
            if (marks[i].crossroad == crossroad) and (marks[i].direction == direction):
                return marks[i].mark
        marks.append(Mark(crossroad, direction, 0))
        return 0

    # increments mark of crossroad and direction.
    # if no such crossroad and direction in marks list, creates such with a mark of 1.
    def markDirection(crossroad, direction):
        for i in range(len(marks)):
            if (marks[i].crossroad == crossroad) and (marks[i].direction == direction):
                marks[i].mark += 1
                return
        marks.append(Mark(crossroad, direction, 1))
        return

    # get marks of all crossroad directions, sorted clockwise, according to current direction
    def getSortedCrossroadMarks(crossroad, direction):
        markForward = -1
        markLeft = -1
        markBackward = -1
        markRight = -1
        if sensorValues[0] < 0:
            markForward = getDirectionMark(crossroad, FORWARD)
        if sensorValues[2] < 0:
            markRight = getDirectionMark(crossroad, RIGHT)
        if sensorValues[4] < 0:
            markBackward = getDirectionMark(crossroad, BACKWARD)
        if sensorValues[6] < 0:
            markLeft = getDirectionMark(crossroad, LEFT)
        if direction == FORWARD:
            return [markLeft, markForward, markRight, markBackward]
        elif direction == RIGHT:
            return [markForward, markRight, markBackward, markLeft]
        elif direction == BACKWARD:
            return [markRight, markBackward, markLeft, markForward]
        else: # LEFT
            return [markBackward, markLeft, markForward, markRight]

    # returns rotated direction by 90 degrees to the left
    def getLeft():
        if direction == FORWARD:
            return LEFT
        elif direction == RIGHT:
            return FORWARD
        elif direction == BACKWARD:
            return RIGHT
        else:
            return BACKWARD

    # returns rotated direction by 90 degrees to the right
    def getRight():
        if direction == FORWARD:
            return RIGHT
        elif direction == RIGHT:
            return BACKWARD
        elif direction == BACKWARD:
            return LEFT
        else:
            return FORWARD

    # returns rotated direction by 180 degrees
    def getBackward():
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
        if msg[1]:
            isFinish = True
        #isFinish = msg[1]

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

       # return values: -1 = goLeft, +1 = goRight , 0 = stayCentered
    def getCentration(forwardDistances):
        lefty = forwardDistances[0]
        righty = forwardDistances[4]
        
        # print(lefty, righty)
        if ((abs(lefty-righty)>threshold) and lefty > 0):
            if (lefty > righty): 
                return 1
            elif (lefty < righty):
                return -1
            else:
                return 0
        else:
            return 0   

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
        global x, y, old_image, old_points, position
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
    quadcopterBaseHandle = client.simxGetObjectHandle('Quadcopter_base', client.simxServiceCall())
    quadcopterHandle = client.simxGetObjectHandle('Quadcopter', client.simxServiceCall())
    bottomProximitySensorHandle=client.simxGetObjectHandle('bottom_proximity_sensor',client.simxServiceCall())
    finish1Handle = client.simxGetObjectHandle('Finish_first', client.simxServiceCall())
    finish2Handle = client.simxGetObjectHandle('Finish_second', client.simxServiceCall())
    finish3Handle = client.simxGetObjectHandle('Finish_third', client.simxServiceCall())
    finish4Handle = client.simxGetObjectHandle('Finish_fourth', client.simxServiceCall())


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

    client.simxCheckCollision(quadcopterHandle[1], finish1Handle[1], client.simxDefaultSubscriber(finishCheck))
    client.simxCheckCollision(quadcopterHandle[1], finish2Handle[1], client.simxDefaultSubscriber(finishCheck))
    client.simxCheckCollision(quadcopterHandle[1], finish3Handle[1], client.simxDefaultSubscriber(finishCheck))
    client.simxCheckCollision(quadcopterHandle[1], finish4Handle[1], client.simxDefaultSubscriber(finishCheck))


    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    client.simxStartSimulation(client.simxDefaultPublisher())

    quadcopterTargetInitMsg = client.simxGetObjectPosition(quadcopterTargetHandle[1], -1, client.simxServiceCall())
    quadcopterTargetPos = quadcopterTargetInitMsg[1]
    
 


    crossRoadProceeded = False
    # main loop 
    while not isFinish:
        correction = 0
        stepSimulation()
        forwardDistances = getForwardDistances()

        # ir strupcels
        if (not crossRoadProceeded and (forwardDistances[0] > 0 and forwardDistances[2] > 0 and forwardDistances[4] > 0)):
            # print('blocker')
            direction = getBackward()
            crossRoadProceeded = True
        
        # if crossroad or turn
        elif (not crossRoadProceeded and isCrossroadOrTurn(forwardDistances)):
            # print('crossroad or turn')
            
            # we can't know is it crossroad or turn till it's middle
            if isCrossroadMiddle():
                # print('middle')
                
                # if crossroad
                if isCrossRoad(forwardDistances):
                    # print('is crossroad')

                    # check if this is new crossroad
                    crossroadIndex = getVisitedCrossroadIndex(position[0], position[1])
                    if crossroadIndex == -1:
                        # crossroad is new
                        crossroadIndex = setVisitedCrossroad(position[0], position[1])
                    print('crossroad number:', crossroadIndex)
                    markDirection(crossroadIndex, getBackward())
                    sortedMarks = getSortedCrossroadMarks(crossroadIndex, direction)
                    print(sortedMarks)
                    
                    # If you arrive at a junction that has no marks (except possibly for the one on the path by which you entered),
                    # choose an arbitrary unmarked path, follow it, and mark it.
                    if (sortedMarks[0] == 0) or (sortedMarks[1] == 0) or (sortedMarks[2] == 0):
                        if sortedMarks[0] == 0:
                            direction = getLeft()
                        elif sortedMarks[2] == 0:
                            direction = getRight()
                        print('zeros', direction)
                        markDirection(crossroadIndex, direction)
                        crossRoadProceeded = True
                    elif (sortedMarks[3] == 1):
                        # If the path you came in on has only one mark, turn around and return along that path, marking it again.
                        # In particular this case should occur whenever you reach a dead end
                        direction = getBackward()
                        print('go back', direction)
                        markDirection(crossroadIndex, direction)
                        crossRoadProceeded = True
                    else:
                        # If not, choose arbitrarily one of the remaining paths with the fewest marks (zero if possible, else one), follow that path, and mark it.
                        if (sortedMarks[0] >= 0) and (sortedMarks[0] < 2):
                            direction = getLeft()
                        elif (sortedMarks[2] >= 0) and (sortedMarks[2] < 2):
                            direction = getRight()
                        print('choose best of remaining', direction)
                        markDirection(crossroadIndex, direction)
                        crossRoadProceeded = True
                
                # if turn
                elif ((forwardDistances[0] < 0) or (forwardDistances[4] < 0)):
                    # print('turn')
                    if (forwardDistances[0] < 0):
                        direction = getLeft()
                    elif (forwardDistances[4] < 0):
                        direction = getRight()
                    crossRoadProceeded = True
        
        # if decision has already been made on this crossroad/turn
        elif (forwardDistances[0] > 0) and (forwardDistances[4] > 0):
            crossRoadProceeded = False
            correction = getCentration(forwardDistances) 
            
        # movement
        if (direction == FORWARD) or (direction == BACKWARD):
            coord = 1
            coordCentration = 0

        else: #left or right
            coord = 0
            coordCentration = 1

        if (direction == FORWARD) or (direction == LEFT):
            correction *= -1
        
        if (direction == BACKWARD) or (direction == LEFT):
            multiplier = -1
        else:
            multiplier = 1
        quadcopterTargetPos[coord] += multiplier * speed 
        quadcopterTargetPos[coordCentration] += correction * speed * correctionCoef
    
        client.simxSetObjectPosition(quadcopterTargetHandle[1], -1, quadcopterTargetPos, client.simxServiceCall())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
        
    # client.simxStopSimulation(client.simxDefaultPublisher())
