import socket
import time
from tkinter import Scale  
import simple_pid
import traceback 
import math
import timeit
import random 
#import ArduinoControl
import rtde_control
import rtde_io
import rtde_receive

#STARTING POSITION: [-200, -442, z , 1.571, 0, 0]
pid = None 
rtde__r = None 
rtde__c = None 
rtde__io = None 
pid = simple_pid.PID(Kp=1, Ki=0, Kd=0)
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
    print(dir(rtde__c))
    print()
    print(dir(rtde__r))
    print()
    print(dir(rtde__io))
    rtde__c.Flags.FLAGS_DEFAULT = rtde__c.FEATURE_TOOL
    rtde__io.setSpeedSlider(.04)
    z_value = float(input("Set TCP Offset in Z-Direction (In Meters)"))
    value = rtde__c.setTcp([0,0,z_value, 0,0,0])
    print("Exiting Startup")

def startingPosition():
    global rtde__r
    global rtde__c
    global rtde__io 
    #DECIDE ON A Z VALUE FOR CONSISTENCY 
    rtde__c.moveL([-200, -442, z , 1.571, 0, 0], asynchronous=True)
    robotMoving = True 
    while robotMoving: 
        TCP_Client.tcp()
        if rtde__c.isSteady():
            robotMoving = False
    #MAYBE PUT IN FREEDRIVE AFTERWARDS? 
def reconnect():
    global rtde__r 
    global rtde__c 
    global rtde__io 
    rtde__c.reconnect()
def moveByVelocity(velocity): 
    global rtde__r 
    global rtde__c 
    global rtde__io 
    global newZ
    #rtde__c.jogStart(velocity, rtde__c.FEATURE_CUSTOM, [.2,.2,.2,0,0,0])
    print("In move method")
    start = time.time()
    print("Velocity: " + str(velocity)) 
    rtde__c.jogStart(velocity, rtde__c.FEATURE_TOOL)
    end = time.time()
    deltaTime = end - start 
    if(deltaTime > .5):
        print("Delay too high")
        quit() 
    print(str(end - start))
    print("Exiting move method")

def PID_Encoder_Robot(): 
    pass

def PID_Rz(Rz, resolution = 1.0): 
    global rtde__r 
    global rtde__c 
    global rtde__io 
    global pid 
    deltaRz = float(Rz) / resolution
    previousRz = float(rtde__r.getActualTCPPose()[5]) 
    for i in range(previousRz, Rz, deltaRz):
        pid.setpoint = i
        new_angle = TCP_Client.tcp()[2] 
        movement = pid(new_angle)
        pose = [0,0,0,0,0,movement]
        pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
        rtde__c.moveL(pose_wrt_base)
        print("I: " + str(i))
        

def PID_Ry(Ry, resolution = 1.0): 
    try: 
        global rtde__r 
        global rtde__c 
        global rtde__io 
        global pid  
        new_angle = TCP_Client.tcp()[1]
        pid.setpoint = new_angle + Ry 
        movement = pid(new_angle)
        previousPosition = rtde__r.getActualTCPPose()[4] 
        movement = movement - previousPosition
        pose = [0,0,0,0,movement,0]
        pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
        rtde__c.moveL(pose_wrt_base, asynchronous=True)
        robotMoving = True 
        while robotMoving: 
            TCP_Client.tcp()
            if rtde__c.isSteady():
                robotMoving = False
    except Exception:
        traceback.print_exc()
        

#IDEA TO MOVE Z: CONTROL WITH DISTANCE MOTOR METHOD CALL FIRST; AND THEN AS THE MOTOR MOVES; MOVE THE ROBOT TO COMPENSATE FOR OFFSET
def translateZ(z, scale, speed=.25, acceleration=1.2): 
    global rtde__r 
    global rtde__c 
    global rtde__io 
    global pid
    tension_pid = Tension_PID() 
    for i in range(0, z, scale): 
        ArduinoControl.controlMotorWithDistance(scale, 1) 
        pose = [0,0,-1 * scale,0,0,0]
        pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
        acceleration = ArduinoControl.getAcceleration()
        speed = ArduinoControl.getCurrentMotorVelocity()
        rtde__c.moveL(pose_wrt_base, speed=speed, acceleration=acceleration)        
        tension_pid.TensionPID(setpoint=1)

def PID_Z(Z, resoltuion=1.0): 
    pass 
def moveByPose(pose, resolution = 1.0): 
    global rtde__r 
    global rtde__c 
    global rtde__io 
    global newZ
    try: 
        pose[2] = -1 * pose[2]
        if float(pose[4]) > 0:
            pose[4] = pose[4] - math.pi
        else:
            pose[4] = pose[4] + math.pi
        if float(pose[5]) > 0:
            pose[5] = pose[5] - math.pi
        else:
            pose[5] = pose[5] + math.pi    
        #check if z, ry, or rz is a number greater than zero; call pid 
        if math.fabs(pose[0]) > 0:
            print("Not yet completed") 
        elif math.fabs(pose[4]) > 0: 
            PID_Ry(pose[4], resolution)
        elif math.fabs(pose[5]) > 0: 
            PID_Rz(pose[5], resolution) 
        else: 
            pass 
    except Exception as e: 
        print(str(e))

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

class MotorRobotSpeed_PID:
    current_time = time.time()
    last_time = current_time
    current_speed = 0 
    pid = simple_pid.PID(Kp=5)

    def updateSpeed(self, speed, dt):
        #distance at time zero is tcp offset  
        speed += speed * dt

    def SpeedPID(self, setPoint):  
        #Desired Torque is setpoint value
        self.pid.setpoint = setPoint 
        #distance is new offset 
        self.current_time = time.time()
        dt = self.current_time - self.last_time 
        self.pid.sample_time = dt
        


# class Tension_PID: 
#     current_time = time.time()
#     last_time = current_time
#     current_tension = float(ArduinoControl.getTension())
#     pid = simple_pid.PID(Kp=5)

#     def updateTension(self, distance, dt):
#         #distance at time zero is tcp offset  
#         distance += distance * dt
#         pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), [0,0,distance,0,0,0])
#         #CHECK BELOW WITH SOMEONE 
#         rtde__c.moveL(pose_wrt_base)
#         rtde__c.setTcp([0,0,distance, 0,0,0])
#         self.current_tension = float(ArduinoControl.getTension())

#     def TensionPID(self, setPoint):  
#         #Desired Torque is setpoint value
#         self.pid.setpoint = setPoint 
#         #distance is new offset 
#         distance = float(rtde__c.getTcpOffset())
#         self.current_time = time.time()
#         dt = self.current_time - self.last_time 
#         self.pid.sample_time = dt
#         distance = self.pid(self.current_tension)
#         self.updateTension(distance, dt)

startup()
import TCP_Client
pose = [0,0,0,0,.2,0]
pose_wrt_base = rtde__c.poseTrans(rtde__c.getForwardKinematics(), pose)
moveByPose(pose, 20)
quit()

