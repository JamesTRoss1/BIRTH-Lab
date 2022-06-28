import dis
from tkinter.messagebox import ABORT
from xmlrpc.client import MAXINT, MININT
import pyfirmata2
import time
import traceback
import math
import simple_pid
#import MotorRobotConditional
import UR_Move_RTDE
import os
import TCP_Client

pid = simple_pid.PID(Kp=4, Ki=1, Kd=.1)
pid.output_limits = (0, 1)
board = None
pin_dir = None
speed = None
pin_a_encoder = None
pin_b_encoder = None
load_cell = None
counter = 0
motor_velocity = 0
position = None
prevA = None
prevB = None
previous_velocity = 0.0
previous_acceleration = 0.0
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
        while True:
            data = pin.read()
            if data is None:
                continue
            else:
                return str(float(data))
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

def controlMotorWithDistance(distance = 0, robotPose = [0,0,0,0,0,0], moveRobot = True) -> float:
    #not async but if distance is low enough it wont matter much
    #lowest distance:
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell, counter
    prevCount = 0
    distanceExpended = 0
    isDone = False
    aLastState = None
    bLastState = None
    start_time = None
    pose = [0,0,0,0,0,0]
    i = 0
    lastState = None
    if distance > 0:
        direction = 0
    else:
        direction = 1
    try:
        distance = math.fabs(distance)
        if distanceExpended == 0:
            print("Distance To Go: " + str(distance))
            start_time = time.time()
            #.25 or lower, it will be really precise
            #Do not do anything above that
            write(.25, board, speed)
            write(direction, board, pin_dir)
            pin_a_encoder.enable_reporting()
            pin_b_encoder.enable_reporting()
            print("Moving Motor")
            TCP_Client.tcp()
            beforeX = TCP_Client.shapeX[-1]
            isMoving = True
            for i in range(0, len(robotPose)):
                pose[i] = pose[i] + robotPose[i]
            #index 1 is facing sink wall
            if direction == 0:
                pose[2] = (-1 * distance) / 100
            else:
                pose[2] = distance / 100
            if moveRobot:
                pose_wrt_base = UR_Move_RTDE.rtde__c.poseTrans(UR_Move_RTDE.rtde__c.getForwardKinematics(), pose)
                #Speed is chosen at 25% motor speed
                #Originally .0337
                UR_Move_RTDE.rtde__c.moveL(pose_wrt_base, speed=.0333, acceleration=3.0, asynchronous=True)
                TCP_Client.tcp()
                print("Moving Robot")
        while not isDone:
            if abs(distanceExpended) >= distance:
                print("Distance: " + str(distance))
                write("0", board, speed)
                isDone = True
                pin_a_encoder.disable_reporting()
                pin_b_encoder.disable_reporting()
                print("Totally Done Moving")
                TCP_Client.tcp()
                afterX = TCP_Client.shapeX[-1]
                print("Distance Verified By FBGS: " + str(float(afterX) - float(beforeX)))
                #LUCAS FILE CODE
                aFile = open("ZResults.txt", "a")
                aFile.write("Distance: " + str(distance) + "\n" + "Distance Verified By FBGS: " + str(float(afterX) - float(beforeX)) + "\n\n")
                aFile.close()
                #-------------------------
                counter = 0
                break
            #     #Must clear tcp  buffer
            else:
                #Compute Distance
                distanceExpended = 1.25 * (counter / (34607)) * (2.0 * math.pi * .43)
            print(str(TCP_Client.tcp()))
        end_time = time.time()
        dt = end_time - start_time
        while isMoving and moveRobot:
            TCP_Client.tcp()
            if UR_Move_RTDE.rtde__c.isSteady():
                isMoving = False
        velocity = (distanceExpended / 100) / dt
        #print(str(TCP_Client.tcp()))
        setCurrentMotorVelocity(velocity / .04)  #.04 is equal to the speed scale of the robot
        print("Motor Velocity: " + str(velocity)) #NOT OFFSET FOR ROBOT YET
        #setAcceleration(((velocity - previous_velocity) / dt) / .04) #.04 is equal to the speed scale of the robot
        return distanceExpended
    except Exception as e:
        traceback.print_exc()

def move():
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell, counter
    write(.25, board, speed)
    write(1, board, pin_dir)

def stop():
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell, counter
    write(0, board, speed)

def pinAIncrementer(value):
    global counter, prevA
    if bool(value) != bool(prevA):
        #print("A Value: " + str(value))
        counter += 1
        prevA = value

def pinBIncrementer(value):
    global counter, prevB
    if bool(value) != bool(prevB):
        #print("B Value: " + str(value))
        counter += 1
        prevB = value


def initialize():
    global board, pin_dir, speed, pin_a_encoder, pin_b_encoder, load_cell
    PORT = pyfirmata2.Arduino.AUTODETECT
    board = pyfirmata2.Arduino(PORT)
    board.samplingOn(1)
    pin_dir = board.get_pin(str("d:9:p"))
    speed = board.get_pin(str("d:10:p"))
    pin_a_encoder = board.get_pin(str("d:2:i"))
    pin_a_encoder.register_callback(pinAIncrementer)
    pin_b_encoder = board.get_pin(str("d:3:i"))
    pin_b_encoder.register_callback(pinBIncrementer)
    load_cell = board.get_pin(str("a:0:i"))
    load_cell.enable_reporting()
    tester = board.get_pin("d:5:i")
    tester.enable_reporting()
    write(0, board, speed)
    #while True:
      #  print(str(pin_a_encoder.read()))
#0 is counter clockwise; 1 is clockwise
def robot_motor_tension(minimumTension = .16, maxTension = .26):
    #determines how much to move by in given cycle
    #Fix this later on
    #Desired Tension: .44
    #Difference: .36

    tension = 1 - float(getTension())
    movement = .01
    needToKeepMoving = True
    robotDistance = UR_Move_RTDE.getCurrentOffset()
    print("Tension: " + str(tension))
    try:
        while needToKeepMoving:
            if tension < minimumTension:
                #COMMENT THE 2 LINES BELOW WHEN NOT IN DEMO
                #stop()
                #f not UR_Move_RTDE.rtde__c.isSteady():
                    #UR_Move_RTDE.rtde__c.stopL()
                UR_Move_RTDE.moveInZDirection(-movement)
                print("Moving Back")
                #Set Robot TCP
                UR_Move_RTDE.rtde__c.setTcp([0,0,robotDistance + (-1 * movement), 0,0,0])
            elif tension > maxTension:
                #COMMENT THE 2 LINES BELOW WHEN NOT IN DEMO
                #stop()
                #if not UR_Move_RTDE.rtde__c.isSteady():
                #    UR_Move_RTDE.rtde__c.stopL()
                UR_Move_RTDE.moveInZDirection(movement)
                print("Moving Forward")
                #Set robot tcp
                UR_Move_RTDE.rtde__c.setTcp([0,0,robotDistance + (1 * movement), 0,0,0])
            else:
                needToKeepMoving = False
                print("Leaving")
                break
            tension = 1 - float(getTension())
            robotDistance = UR_Move_RTDE.getCurrentOffset()
            TCP_Client.tcp()
            # if(desiredTether < tether - motor_precision):
            #     motorDistance = motorDistance - motor_correction
            # elif(desiredTether > tether + motor_precision):
            #     motorDistance = motorDistance + motor_correction
            # else:
            #     pass
    except Exception:
        pass


def testingStart():
    initialize()
    input("Turn on the motor switch now. Press enter to continue.")
    UR_Move_RTDE.startup()
    testingMotorCell(1, .1)

# speedVar = None
# print("Done")
# speedVar = int(str(input("Speed: ")).strip())
# dirVar = int(str(input("Direction: ")).strip())
# rotation = int(str(input("Rotation: ")).strip())
# write(speedVar, board, speed)
# write(dirVar, board, pin_dir)
# time.sleep(5)
# write(0, board, speed)
# #29.7 cm in 35.6/4 seconds at full power
#controlMotor(pin_a_encoder, pin_b_encoder, speed, "0", None, None, float(rotation), board, int(dirVar))
