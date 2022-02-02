from xmlrpc.client import MAXINT, MININT
from pyfirmata import Arduino, util
import time 
import traceback
import TCP_Client
import math 
import os 
board = None 
pin_dir = None
speed = None 
pin_a_encoder = None
pin_b_encoder = None
load_cell = None
motor_velocity = 0 
position = None 
previous_velocity = 0.0 
acceleration = 0.0

def getLoadCell():
    global load_cell
    return load_cell

def getBoard():
    global board
    return board 

def start(path):
    board = Arduino(str(path))
    return board 

def setPosition(pose):
	global position 
	position = pose

def getPosition():
	global position
	return position

def setCurrentMotorVelocity(vel): 
    global motor_velocity
    motor_velocity = vel 

def getCurrentMotorVelocity() -> float:
    global motor_velocity
    return motor_velocity

def getAcceleration() -> float:
    global acceleration
    return acceleration

def setAcceleration(accel): 
    global acceleration
    acceleration = accel

def read(pin, board):
    try:
        data = pin.read()
        return str(data) 
    except Exception:
        traceback.print_exc()
        return "Read Error"

def getTension():
    global load_cell
    global board
    return str(read(load_cell, board))

def write(message, board, pin):
    try:
        pin.write(float(str(message)))
        return True 
    except Exception:
        traceback.print_exc()
        return False 
#This method takes a channelA and channelB to listen on; if there is a change it records the change as either a 1 or -1. No change is defined as a 0. 
#Takes optional parameter to define aLastState
#Returns two integers with the first being the rotation and the second being a counter 
#Default Radius: 
def readPosition(channelA = None, channelB = None, counter = 0, aLastState = None, board = None, bLastState = None, direction = None, radius = 0):
    print("UPDATE RADIUS OR THIS WILL NOT WORK")
    aState = str(read(channelA, board))
    bState = str(read(channelB, board))
    #Not Updated
    try:
        if(aState != aLastState or bState != bLastState):
            if int(direction) == 1:
                counter = counter + 1
            if int(direction) == 0:
                counter = counter - 1
    except Exception:
        traceback.print_exec()
    finally: 
        aLastState = str(read(channelA, board))
        bLastState = str(read(channelB, board))
        #as long as counter continues being fed; distance expended will be exact amount; NOT DELTA DISTANCE
        distanceExpended = (counter / (34607 / 2)) * (2 * math.PI * radius) 
        setPosition(distanceExpended)
        return aLastState, bLastState, counter, distanceExpended
#Motor Control
#pin 2 and 3 encoder; input; digital
#Number of counts is how many revolutions the motor should do; for closed-loop make this number small (maybe like 100) 
def controlMotor(channelA, channelB, writeChannel, message, numberOfCounts = None, counter = 0, revolution = None, board = None, direct = None):
    #Has not reached desired rotation
    start_time = time.time() 
    fullCycle = int(34607 / 2)
    counter = 0
    isDone = False
    aLastState = None
    bLastState = None
    direction = direct
    try: 
        if numberOfCounts is None: 
            numberOfCounts = float(revolution) * fullCycle  
        while not(isDone):
            print(str(read(getLoadCell(), getBoard())))
            if abs(counter) >= numberOfCounts:
                write(str(message), board, writeChannel)
                isDone = True
            if counter < numberOfCounts:
                aLastState, bLastState, counter, distanceExpended = readPosition(channelA=channelA, channelB= channelB,counter=counter, board=board, aLastState = aLastState, bLastState = bLastState, direction = direction)
                print(str(counter))
                #Clear TCP Buffer 
                TCP_Client.tcp()
        end_time = time.time()
        dt = end_time - start_time
        velocity = distanceExpended / dt
        global previous_velocity 
        setCurrentMotorVelocity(velocity)
        setAcceleration((velocity - previous_velocity) / dt)
        previous_velocity = velocity
        return distanceExpended
    except Exception: 
        return MININT

def controlMotorWithDistance(distance = 0, desiredSpeed = 0) -> float:
    #not async but if distance is low enough it wont matter much 
    #lowest distance: 
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell
    counter = 0
    distanceExpended = 0
    isDone = False
    aLastState = None
    bLastState = None
    start_time = None 
    if distance > 0: 
        direction = 1 
    else: 
        direction = 0
    try: 
        if distanceExpended == 0: 
            start_time = time.time() 
            write(desiredSpeed, board, speed)
            write(direction, board, pin_dir)        
        while not(isDone):
            if abs(distanceExpended) >= distance:
                write(0.0, board, speed)
                isDone = True 
            if distance < distanceExpended:
                aLastState, bLastState, counter, instantDistanceExpended = readPosition(channelA=pin_a_encoder, channelB=pin_b_encoder,counter=counter, board=board, aLastState = aLastState, bLastState = bLastState, direction = direction)
                distanceExpended = instantDistanceExpended
        end_time = time.time()
        dt = end_time - start_time
        velocity = distanceExpended / dt 
        global previous_velocity 
        setCurrentMotorVelocity(velocity)
        setAcceleration((velocity - previous_velocity) / dt)
        previous_velocity = velocity
        setCurrentMotorVelocity(velocity)
        return distanceExpended
    except Exception: 
        return MININT

def asyncControlMotorWithDistance(channelA, channelB, distance = 0, desiredSpeed = 0, aLastState=None, bLastState=None, distanceExpended=0):
    #Has not reached desired rotation
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell
    start_time = time.time() 
    counter = 0
    isDone = False
    if distance > 0: 
        direction = 1 
    else: 
        direction = 0
    try: 
        if distance == 0: 
            write(desiredSpeed, board, speed)
            write(direction, board, pin_dir)       
        if abs(distanceExpended) >= distance:
            write(0.0, board, speed)
            isDone = True 
        if distance < distanceExpended:
            aLastState, bLastState, counter, distanceExpended = readPosition(channelA=channelA, channelB= channelB,counter=counter, board=board, aLastState = aLastState, bLastState = bLastState, direction = direction)
        end_time = time.time()
        dt = end_time - start_time
        velocity = distanceExpended / dt 
        setCurrentMotorVelocity(velocity)
        return distance + distanceExpended, isDone, aLastState, aLastState
    except Exception:
        print("Exception on control motor") 
        return 0, False, 0, 0

def initialize():
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell
    board = start("/dev/ttyACM0")
    it = util.Iterator(board)
    it.start()
    pin_dir = board.get_pin(str("d:9:p"))
    speed = board.get_pin(str("d:10:p")) 
    pin_a_encoder = board.get_pin(str("d:2:i"))
    pin_b_encoder = board.get_pin(str("d:3:i"))
    load_cell = board.get_pin(str("a:0:i"))
#0 is counter clockwise; 1 is clockwise 
initialize()
speedVar = None
print("Done")
speedVar = str(input("Speed: ")).strip()
dirVar = str(input("Direction: ")).strip()
rotation = str(input("Rotation: ")).strip()
write(speedVar, board, speed)
write(dirVar, board, pin_dir)
#29.7 cm in 35.6/4 seconds at full power
controlMotor(pin_a_encoder, pin_b_encoder, speed, "0", None, None, float(rotation), board, int(dirVar))