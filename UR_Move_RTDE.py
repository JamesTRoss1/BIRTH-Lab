from os import kill
import socket
import time
from tkinter import Scale
from turtle import degrees
import simple_pid
#from scipy.spatial.transform import Rotation as R
import traceback
import math
#import ArduinoControl2
import timeit
import random
import rtde_control
import decimal
import rtde_io
import rtde_receive
import TCP_Client

#import MotorRobotConditional

#STARTING POSITION: [-200, -442, z , 1.571, 0, 0]
pid = None
rtde__r = None
rtde__c = None
rtde__io = None
pid = simple_pid.PID(Kp=1, Ki=0, Kd=0)
ry_pid = simple_pid.PID(Kp=1, Ki=0, Kd=0)
rz_pid = simple_pid.PID(Kp=1, Ki=0, Kd=0)
newZ = 0
def startup():
    global pid
    global rtde__r
    global rtde__c
    global rtde__io
    print("Entering Startup")
    HOST = "192.168.1.101" #UR MACHINE
    PORT = 30002
    rtde__r = rtde_receive.RTDEReceiveInterface(str(HOST))
    rtde__c = rtde_control.RTDEControlInterface(str(HOST))
    rtde__io = rtde_io.RTDEIOInterface(str(HOST))
    print("RTDE__R \n")
    print(dir(rtde__r))
    print()
    print("RTDE__C \n")
    print(dir(rtde__c))
    print()
    print("RTDE__IO \n")
    print(dir(rtde__io))
    rtde__c.Flags.FLAGS_DEFAULT = rtde__c.FEATURE_TOOL
    rtde__io.setSpeedSlider(.04)
    z_value = float(input("Set TCP Offset in Z-Direction (In Meters)"))
    value = rtde__c.setTcp([0,0,z_value, 0,0,0])
    #Ry_Incremental3(1.5)
    print("Exiting Startup")

def getCurrentOffset():
    global rtde__c
    #Offset returns list of 6 elements. But we only need z coord
    return float(rtde__c.getTCPOffset()[2])

def moveInZDirection(z):
    pose = [0,0,z / 100,0,0,0] # z is in cm
    pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
    rtde__c.moveL(pose_wrt_base, asynchronous=True)
    robotMoving = True
    while robotMoving:
        TCP_Client.tcp()
        if rtde__c.isSteady():
            robotMoving = False
def moveInRZ(Rz):
    Rz = math.radians(Rz)
    pose = [0,0,0,Rz,0,0] # z is in cm
    pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
    rtde__c.moveL(pose_wrt_base, asynchronous=True)
    robotMoving = True
    while robotMoving:
        TCP_Client.tcp()
        if rtde__c.isSteady():
            robotMoving = False

def moveInRY(Ry):
    Ry = math.radians(Ry)
    pose = [0,0,0,0,Ry,0] # z is in cm
    pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
    rtde__c.moveL(pose_wrt_base, asynchronous=True)
    robotMoving = True
    while robotMoving:
        TCP_Client.tcp()
        if rtde__c.isSteady():
            robotMoving = False

def startingPosition(): # Moving to home position
    global rtde__r
    global rtde__c
    global rtde__io
    #DECIDE ON A Z VALUE FOR CONSISTENCY
    rtde__c.moveL([-215, -582.21, 91.42 , 1.271, -1.182, 1.135], asynchronous=True)
    robotMoving = True
    while robotMoving:
        TCP_Client.tcp()
        if rtde__c.isSteady():
            robotMoving = False
    #PUT IN FREEDRIVE AFTERWARDS
    print("Robot is in FreeDrive To Ensure Correct Position")
    rtde__r.teachMode()
    input("Press Any Key To End Freedrive")
    rtde__r.endTeachMode()

def reconnect():
    global rtde__r
    global rtde__c
    global rtde__io
    rtde__c.reconnect()

def Ry_Incremental3(Ry): #JPID for RY
    try:
        global rtde__r
        global rtde__c
        global rtde__io
        new_angleRy = float(TCP_Client.tcp()[2]) #New angle should be angle of needle, measured in radians
        print("Starting Angle: " + str(new_angleRy))
        desiredAngleRy =  new_angleRy + math.radians(Ry)
        changeRy = math.radians(Ry)
        CHANGE = changeRy
        print("DA CHANGE: " + str(changeRy))
        errorRy = 0
        while math.fabs(new_angleRy - desiredAngleRy) > .01:
            try:
                changeRy = changeRy / 2
                robotChangeRy = changeRy + errorRy
                print("Desired Angle", desiredAngleRy)
                print("New Angle", new_angleRy)
                print("Robot Change: ", robotChangeRy)
                print('changeRy :',changeRy)

                if desiredAngleRy > new_angleRy and desiredAngleRy > 0:
                    print("1")
                    movementRy = robotChangeRy
                if desiredAngleRy < new_angleRy and desiredAngleRy > 0:
                    print("2")
                    movementRy = robotChangeRy
                if desiredAngleRy > new_angleRy and desiredAngleRy < 0:
                    print("3")
                    movementRy = robotChangeRy
                if desiredAngleRy < new_angleRy and desiredAngleRy < 0:
                    print("4")
                    movementRy = robotChangeRy
                print(movementRy)

                pose = [0,0,0,0,-movementRy,0]
                pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
                rtde__c.moveL(pose_wrt_base, acceleration=3, speed=3, asynchronous=True)
                # start = time.time()
                while not rtde__c.isSteady():
                    checkRy = TCP_Client.tcp()[2]
                    if Ry > 0 and (checkRy - desiredAngleRy > 0 and checkRy - desiredAngleRy <= .008):
                        #up decel if accuracy is needed
                        rtde__c.stopL(1)
                        print("STOPPING. POGGERS")
                        print(str(TCP_Client.tcp()[2] - desiredAngleRy))
                        return True

                    if Ry < 0 and (checkRy - desiredAngleRy < 0 and checkRy - desiredAngleRy >= -.008):
                        rtde__c.stopL(1)
                        print("STOPPING. POGGERS")
                        print(str(TCP_Client.tcp()[2] - desiredAngleRy))
                        return True
                    TCP_Client.tcp()[2]
                delAngleRy = TCP_Client.tcp()[2] - new_angleRy
                if math.fabs(CHANGE) >= math.fabs(16 * changeRy):
                    print("NEW CHANGE TECHNIQUE")
                    errorRy = desiredAngleRy - TCP_Client.tcp()[2] - errorRy
                else:
                    errorRy = changeRy - delAngleRy - errorRy
                new_angleRy = TCP_Client.tcp()[2]

                print("TCP: " + str(TCP_Client.tcp()))
                print("Error: ", errorRy)
            except Exception:
                traceback.print_exc()
                continue
                #new_angle = TCP_Client.tcp()[1] #Maybe not needed bc of the low net change of movement per iteration
    except Exception:
        traceback.print_exc()
    finally:
        error = math.fabs(math.fabs(new_angleRy) - math.fabs(desiredAngleRy))
        return error

def Ry_Iterative(Ry, steps): #JPID for RY
    try:
        global rtde__r
        global rtde__c
        global rtde__io
        #Ry = -Ry
        new_angleRy = float(TCP_Client.tcp()[2]) #New angle should be angle of needle, measured in radians
        print("Starting Angle: " + str(new_angleRy))
        desiredAngleRy =  new_angleRy + math.radians(Ry)
        changeRy = math.radians(Ry) / steps
        theoRy = changeRy
        actualRy = None
        Kp = 1
        print("DA CHANGE: " + str(changeRy))
        errorRy = 0
        for step in range(1, steps + 1):
            try:
                # if actualRy == None:
                #     robotChangeRy = errorRy * Kp + Kp * changeRy
                # else:
                #     Kp = TCP_Client.tcp()[2] / (changeRy * step)
                #     robotChangeRy = errorRy * Kp + Kp * changeRy
                beforeMove = TCP_Client.tcp()[2]
                print("BEFORE: " + str(beforeMove))
                print("Desired Angle", desiredAngleRy)
                print('changeRy :',changeRy)
                if errorRy == 0:
                    robotChangeRy = Kp * changeRy
                print("Robot Change: ", robotChangeRy)
                movementRy = robotChangeRy

                pose = [0,0,0,0,-movementRy,0]
                pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
                rtde__c.moveL(pose_wrt_base, acceleration=3, speed=3, asynchronous=True)
                # start = time.time()
                while not rtde__c.isSteady():
                    checkRy = TCP_Client.tcp()[2]
                    if Ry > 0 and (checkRy - desiredAngleRy > 0 and checkRy - desiredAngleRy <= .015):
                        rtde__c.stopL(10)
                        print("STOPPING. POGGERS")
                        return True

                    if Ry < 0 and ((checkRy - desiredAngleRy < 0) and (checkRy - desiredAngleRy >= -.015)):
                        rtde__c.stopL(10)
                        print("STOPPING. POGGERS")
                        return True

                    TCP_Client.tcp()[2]
                afterMove = TCP_Client.tcp()[2]
                print("AFTER: " + str(afterMove))
                delAngleRy = math.fabs(afterMove - beforeMove)
                print("Delta Angle" + str(delAngleRy))
                print("Step: " + str(step))
                errorRy = changeRy - delAngleRy + errorRy
                Kp = 1.0 / (delAngleRy / changeRy)
                print("Kp: " + str(Kp))
                robotChangeRy = (errorRy * Kp) + (Kp * changeRy)
                print("change ry: " + str(changeRy))
                print("Error: ", errorRy)
                print(str(TCP_Client.tcp()))
            except Exception:
                traceback.print_exc()
                continue
                #new_angle = TCP_Client.tcp()[1] #Maybe not needed bc of the low net change of movement per iteration
    except Exception:
        traceback.print_exc()
    finally:
        error = math.fabs(math.fabs(new_angleRy) - math.fabs(desiredAngleRy))
        return error

def Rz_Incremental(Rz):# JPID RZ
    try:
        global rtde__r
        global rtde__c
        global rtde__io
        global pid
        Rz_new_angle = float(TCP_Client.tcp()[1]) #New angle should be angle of needle, measured in radians
        print("Starting Angle: " + str(Rz_new_angle))
        desiredRzAngle =  Rz_new_angle + math.radians(Rz)
        changeRz = math.radians(Rz)
        CHANGE = changeRz
        print("DA CHANGE: " + str(changeRz))
        errorRz = 0
        while math.fabs(Rz_new_angle - desiredRzAngle) > .01:
            try:
                changeRz = changeRz / 2
                robotChangeRz = changeRz + errorRz
                print("Desired Angle", desiredRzAngle)
                print("New Angle", Rz_new_angle)
                print("Robot Change: ", robotChangeRz)
                print("Change: ", changeRz)
                if desiredRzAngle > Rz_new_angle and robotChangeRz < 0:
                    print("1")
                    movementRz = robotChangeRz
                if desiredRzAngle < Rz_new_angle and robotChangeRz < 0:
                    print("2")
                    movementRz = -robotChangeRz
                if desiredRzAngle > Rz_new_angle and robotChangeRz > 0:
                    print("3")
                    movementRz = -robotChangeRz
                if desiredRzAngle < Rz_new_angle and robotChangeRz > 0:
                    print("4")
                    movementRz = robotChangeRz
                print("Movement" , movementRz)


                #Rz calculations
                pose = [0,0,0,movementRz,0,0]
                pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
                rtde__c.moveL(pose_wrt_base, acceleration=3, speed=3, asynchronous=True)
                # start = time.time()
                while not rtde__c.isSteady():
                    checkRz = TCP_Client.tcp()[1]
                    if Rz > 0 and (checkRz - desiredRzAngle > 0 and checkRz - desiredRzAngle <= .008):
                        rtde__c.stopL(10)
                        print("STOPPING. POGGERS")
                        print(str(checkRz - desiredRzAngle))
                        return True

                    if Rz < 0 and (checkRz - desiredRzAngle < 0 and checkRz - desiredRzAngle >= -.008):
                        rtde__c.stopL(10)
                        print("STOPPING. POGGERS")
                        print(str(checkRz - desiredRzAngle))
                        return True
                    TCP_Client.tcp()[1]

                delAngleRz = TCP_Client.tcp()[1] - Rz_new_angle
                if math.fabs(CHANGE) >= math.fabs(16 * changeRz):
                    print("NEW CHANGE TECHNIQUE")
                    errorRz = -(desiredRzAngle - TCP_Client.tcp()[1])
                else:
                    errorRz = -(changeRz - delAngleRz - errorRz)
                Rz_new_angle = TCP_Client.tcp()[1]
                print("Error: ", errorRz)

            except Exception:
                traceback.print_exc()
                continue
                #new_angle = TCP_Client.tcp()[1] #Maybe not needed bc of the low net change of movement per iteration
    except Exception:
        traceback.print_exc()
    finally:
        error = math.fabs(math.fabs(Rz_new_angle) - math.fabs(desiredRzAngle))
        return error

def omniDirectional_Iterative(z, Ry, Rz, steps=10, motor=True): #JPID RY RZ
    try:
        global rtde__r
        global rtde__c
        global rtde__io
        zDistance = float(z)
        stepZ = zDistance / steps
        Rz_new_angle = float(TCP_Client.tcp()[1]) #New angle should be angle of needle, measured in radians
        print("RZ NEW ANGLE: " + str(TCP_Client.tcp()))
        desiredRzAngle =  Rz_new_angle + math.radians(Rz)
        errorRz = 0
        changeRz = math.radians(Rz)
        stepRz = changeRz / steps
        new_angleRy = float(TCP_Client.tcp()[2]) #New angle should be angle of needle, measured in radians
        desiredAngleRy =  new_angleRy + math.radians(Ry)
        print("NEW ANGLE RY: " + str(TCP_Client.tcp()))
        changeRy = math.radians(Ry)
        stepRy = changeRy / steps
        traveled = 0
        errorRy = 0
        step = 1
        # Test this one first: while step < steps + 1:
        while math.fabs(moveAngles[1] - desiredRzAngle) > .02 and math.fabs(moveAngles[2] - desiredAngleRy) > .02:
            print("Rz_new_angle ", Rz_new_angle)
            print("Desired Rz Angle ", desiredRzAngle)
            print("Desired Ry Angle", desiredAngleRy)
            print("TCP IN LOOP, ",  str(TCP_Client.tcp()))
            print("New Angle Ry ", new_angleRy)
            print("Difference Ry: " + str((math.fabs(new_angleRy - desiredAngleRy))))
            print("Difference Rz: " + str(math.fabs(Rz_new_angle - desiredRzAngle)))
            movementRy = 0.0
            movementRz = 0.0
            try:
                if math.fabs(Rz_new_angle - desiredRzAngle) > .03:
                    changeRz = stepRz
                    if step >= steps + 1:
                        changeRz = 0
                    robotChangeRz = changeRz + errorRz
                    if desiredRzAngle > Rz_new_angle and robotChangeRz < 0:
                        movementRz = robotChangeRz
                    if desiredRzAngle < Rz_new_angle and robotChangeRz < 0:
                        movementRz = -robotChangeRz
                    if desiredRzAngle > Rz_new_angle and robotChangeRz > 0:
                        movementRz = -robotChangeRz
                    if desiredRzAngle < Rz_new_angle and robotChangeRz > 0:
                        movementRz = robotChangeRz
                else:
                    movementRz = 0.0

                if math.fabs(new_angleRy - desiredAngleRy) > .03:
                    changeRy = stepRy
                    if step >= steps + 1:
                        changeRy = 0
                    robotChangeRy = changeRy + errorRy
                    if desiredAngleRy > new_angleRy and desiredAngleRy > 0:
                        movementRy = robotChangeRy
                    if desiredAngleRy < new_angleRy and desiredAngleRy > 0:
                        movementRy = robotChangeRy
                    if desiredAngleRy > new_angleRy and desiredAngleRy < 0:
                        movementRy = robotChangeRy
                    if desiredAngleRy < new_angleRy and desiredAngleRy < 0:
                        movementRy = robotChangeRy
                else:
                    movementRy = 0.0

                angles = TCP_Client.tcp()
                Rz_new_angle = angles[1]
                new_angleRy = angles[2]
                #Ry is left to right movement
                #Rz is up and down movement
                pose = [0,0,0,movementRz,movementRy,0]
                # start = time.time()

                #clear tcp buffer inside control motor with distance
                pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
                rtde__c.moveL(pose_wrt_base, speed=3, acceleration=3.0, asynchronous=True)
                while not rtde__c.isSteady():
                    moveAngles = TCP_Client.tcp()
                    if math.fabs(moveAngles[1] - desiredRzAngle) < .015 and math.fabs(moveAngles[2] - desiredAngleRy) < .015:
                        print("Angles Reached")
                        rtde__c.stopL()

                delAngleRz = TCP_Client.tcp()[1] - Rz_new_angle
                if step < steps + 1:
                    errorRz = changeRz - delAngleRz + errorRz
                else:
                    errorRz = desiredRzAngle - Rz_new_angle + errorRz
                print("Error Rz: ", errorRz)

                delAngleRy = TCP_Client.tcp()[2] - new_angleRy
                if step < steps + 1:
                    errorRy = changeRy - delAngleRy + errorRy
                else:
                    errorRy = desiredAngleRy - new_angleRy + errorRy
                print("Error Ry: ", errorRy)
                print("TCP BEFORE: " + str(TCP_Client.tcp()))
                tcp_angles = TCP_Client.tcp()
                beforeAngleRy = tcp_angles[2]
                beforeAngleRz = tcp_angles[1]
                if step >= steps + 1:
                    pass
                else:
                    if motor and z != 0:
                        traveled += ArduinoControl2.controlMotorWithDistance(stepZ)
                print("TCP AFTER: " + str(TCP_Client.tcp()))
                tcp_angles = TCP_Client.tcp()
                afterAngleRy = tcp_angles[2]
                afterAngleRz = tcp_angles[1]
                diffAngleRy = afterAngleRy - beforeAngleRy
                diffAngleRz = afterAngleRz - beforeAngleRz
                desiredAngleRy = desiredAngleRy + diffAngleRy
                desiredRzAngle = desiredRzAngle + diffAngleRz

                # angles = TCP_Client.tcp()
                # Rz_new_angle = angles[1]
                # new_angleRy = angles[2]
                print("Distance: ", traveled)
                step = step + 1
            except Exception:
                traceback.print_exc()
                continue
                #new_angle = TCP_Client.tcp()[1] #Maybe not needed bc of the low net change of movement per iteration
    except Exception:
        traceback.print_exc()
    finally:
        errorRz = math.fabs(Rz_new_angle - desiredRzAngle)
        errorRy = math.fabs(new_angleRy - desiredAngleRy)
        netDistance = z - traveled

        if motor:
            if netDistance > 0:
                ArduinoControl2.controlMotorWithDistance(netDistance)
            ArduinoControl2.robot_motor_tension()
        return errorRy, errorRz

def omniDirectional_SuperTask(z, Ry, Rz, motor=True):
    try:
        global rtde__r
        global rtde__c
        global rtde__io
        zDistance = float(z)
        Rz_new_angle = float(TCP_Client.tcp()[1]) #New angle should be angle of needle, measured in radians
        print("RZ NEW ANGLE: " + str(TCP_Client.tcp()))
        desiredRzAngle =  Rz_new_angle + math.radians(Rz)
        errorRz = 0
        changeRz = math.radians(Rz)
        new_angleRy = float(TCP_Client.tcp()[2]) #New angle should be angle of needle, measured in radians
        desiredAngleRy =  new_angleRy + math.radians(Ry)
        print("NEW ANGLE RY: " + str(TCP_Client.tcp()))
        changeRy = math.radians(Ry)
        traveled = 0
        errorRy = 0
        while (math.fabs(Rz_new_angle - desiredRzAngle) > .05) or (math.fabs(new_angleRy - desiredAngleRy) > .05):
            print("Rz_new_angle ", Rz_new_angle)
            print("Desired Rz Angle ", desiredRzAngle)
            print("Desired Ry Angle", desiredAngleRy)
            print("TCP IN LOOP, ",  str(TCP_Client.tcp()))
            print("New Angle Ry ", new_angleRy)
            print("Difference Ry: " + str((math.fabs(new_angleRy - desiredAngleRy))))
            print("Difference Rz: " + str(math.fabs(Rz_new_angle - desiredRzAngle)))
            movementRy = 0.0
            movementRz = 0.0
            try:
                changeRz = changeRz / 2
                if math.fabs(Rz_new_angle - desiredRzAngle) > .05:
                    robotChangeRz = changeRz + errorRz
                    if desiredRzAngle > Rz_new_angle and robotChangeRz < 0:
                        movementRz = robotChangeRz
                    if desiredRzAngle < Rz_new_angle and robotChangeRz < 0:
                        movementRz = -robotChangeRz
                    if desiredRzAngle > Rz_new_angle and robotChangeRz > 0:
                        movementRz = -robotChangeRz
                    if desiredRzAngle < Rz_new_angle and robotChangeRz > 0:
                        movementRz = robotChangeRz
                else:
                    movementRz = 0.0

                changeRy = changeRy / 2
                if math.fabs(new_angleRy - desiredAngleRy) > .05:
                    robotChangeRy = changeRy + errorRy
                    if desiredAngleRy > new_angleRy and desiredAngleRy > 0:
                        movementRy = robotChangeRy
                    if desiredAngleRy < new_angleRy and desiredAngleRy > 0:
                        movementRy = robotChangeRy
                    if desiredAngleRy > new_angleRy and desiredAngleRy < 0:
                        movementRy = robotChangeRy
                    if desiredAngleRy < new_angleRy and desiredAngleRy < 0:
                        movementRy = robotChangeRy
                else:
                    movementRy = 0.0

                angles = TCP_Client.tcp()
                Rz_new_angle = angles[1]
                new_angleRy = angles[2]
                #Ry is left to right movement
                #Rz is up and down movement
                pose = [0,0,0,movementRz,movementRy,0]
                # start = time.time()

                zDistance = zDistance / 2
                #clear tcp buffer inside control motor with distance
                pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
                rtde__c.moveL(pose_wrt_base, speed=3, acceleration=3.0, asynchronous=True)
                while not rtde__c.isSteady():
                    TCP_Client.tcp()

                delAngleRz = TCP_Client.tcp()[1] - Rz_new_angle
                errorRz = changeRz - delAngleRz + errorRz
                print("Error Rz: ", errorRz)

                delAngleRy = TCP_Client.tcp()[2] - new_angleRy
                errorRy = changeRy - delAngleRy + errorRy
                print("Error Ry: ", errorRy)

                print("TCP BEFORE: " + str(TCP_Client.tcp()))
                tcp_angles = TCP_Client.tcp()
                beforeAngleRy = tcp_angles[2]
                beforeAngleRz = tcp_angles[1]

                if zDistance > .003 and motor:
                    traveled += ArduinoControl2.controlMotorWithDistance(zDistance)
                print("TCP AFTER: " + str(TCP_Client.tcp()))
                tcp_angles = TCP_Client.tcp()
                afterAngleRy = tcp_angles[2]
                afterAngleRz = tcp_angles[1]
                diffAngleRy = afterAngleRy - beforeAngleRy
                diffAngleRz = afterAngleRz - beforeAngleRz
                #desiredAngleRy = desiredAngleRy + diffAngleRy
                #desiredRzAngle = desiredRzAngle + diffAngleRz
                angles = TCP_Client.tcp()
                Rz_new_angle = angles[1]
                new_angleRy = angles[2]
                print("Distance: ", traveled)
                if(Ry == 0):
                    new_angleRy = desiredAngleRy
                if(Rz == 0):
                    Rz_new_angle = desiredRzAngle

            except Exception:
                traceback.print_exc()
                continue
                #new_angle = TCP_Client.tcp()[1] #Maybe not needed bc of the low net change of movement per iteration
    except Exception:
        traceback.print_exc()
    finally:
        errorRz = math.fabs(math.fabs(Rz_new_angle) - math.fabs(desiredRzAngle))
        errorRy = math.fabs(math.fabs(new_angleRy) - math.fabs(desiredAngleRy))
        netDistance = z - traveled
        if netDistance > 0 and motor:
            ArduinoControl2.controlMotorWithDistance(netDistance)
        #ArduinoControl2.robot_motor_tension()
        return errorRy, errorRz


def stopMoving():
    global rtde__r
    global rtde__c
    global rtde__io
    print("Entering stop move method")
    start = time.time()
    rtde__c.jogStop()
    end = time.time()
    deltaTime = end - start
    if(deltaTime > .5):
        print("Delay too high")
        quit()
    print(str(end - start))
    print("Exiting stop move method")

def quit():
    global rtde__c
    rtde__c.stopScript()
    rtde__c.forceModeStop()
    rtde__c.disconnect()
    print("Stopping Robot")

def rotvec2rpy(rotvec, degreesFlag=True):
    r = R.from_rotvec(list(rotvec))
    return list(r.as_euler('zyx', degrees=bool(degreesFlag)))

def rpy2rotvec(rpy, degreesFlag=True):
    r = R.from_euler('zyx', rpy, degrees=bool(degreesFlag))
    return list(r.as_rotvec())

def float_range(start, stop, step):
  while start < stop:
    yield float(start)
    start += decimal.Decimal(step)

def listRound(values, precision):
    for value in range(0, len(values)):
        values[value] = round(values[value], precision)
    return values

def movementRy(angle): # Lucas
    file1 = open("RyResults_JPID1.txt","a")
    file1.write("\n FBGS angle prior to rotation by " + str(angle) + "degrees \n")
    print(str(TCP_Client.tcp()))
    file1.write(str(TCP_Client.tcp()))
    #angle = math.radians(angle)
    Ry_Incremental3(angle)
    # pose = [0,0,0,0,angle,0]
    # pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
    # rtde__c.moveL(pose_wrt_base, acceleration=3, speed=3,asynchronous=True)
    # robotMoving = True
    # while robotMoving:
    #     TCP_Client.tcp()
    #     if rtde__c.isSteady():
    #         robotMoving = False
    file1.write("\n FBGS angle after rotation by " + str(angle) + "degrees \n")
    print(str(TCP_Client.tcp()))
    file1.write(str(TCP_Client.tcp()))
    file1.close()


def movementRz(angle):#Lucas
    file2 = open("RzResults_JPID1.txt","a")
    file2.write("\n FBGS angle prior to rotation by " + str(angle) + "degrees \n")
    print(str(TCP_Client.tcp()))
    file2.write(str(TCP_Client.tcp()))
    # angle = math.radians(angle)
    # pose = [0,0,0,angle,0,0]
    # pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
    # rtde__c.moveL(pose_wrt_base, acceleration=3, speed=3,asynchronous=True)
    # robotMoving = True
    Rz_Incremental(angle)
    # while robotMoving:
    #     TCP_Client.tcp()
    #     if rtde__c.isSteady():
    #         robotMoving = False
    file2.write("\n FBGS angle after rotation by " + str(angle) + "degrees \n")
    print(str(TCP_Client.tcp()))
    file2.write(str(TCP_Client.tcp()))
    file2.close()
