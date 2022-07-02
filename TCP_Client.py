
from operator import contains
import socket
import sys
import time 
import traceback
import re
from tracemalloc import start
from scipy.spatial.transform import Rotation as R 
import math
from stl import mesh 
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import offsetbox, pyplot 
import os 
import typing 
import math 
import pathlib

#UR_Move_RTDE.startup()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Curvature = []
curvatureAngle = []
shapeX = []
counter = 0
graphUtil = []
shapeY = []
shapeZ = []
startX = 0
startY = 0
desiredSlopeX = 0
desiredSlopeY = 0
desiredSlopeZ = 0
startZ = 0
endX = 0
endY = 0
endZ = 0
previousSlopeIndex = 0 


# Connect the socket to the port where the server is listening
def startup():
    server_address = ('127.0.0.1', 5000)
    print('connecting to {} port {}'.format(*server_address))
    sock.connect(server_address)
    print("connected")


"""Through knowing a slope to look for, you can determine how much translational movement a given fbgs line is into the gel by determining the starting index inside the gel"""
def needleIndexInsideGel(desiredSlopeX: float, desiredSlopeY: float, desiredSlopeZ: float): 
    global previousSlopeIndex
    global shapeX
    global shapeY
    global shapeZ
    tolerance = .05 #Percentage of tolerance used in slope calculation 
    foundSlopeIndices = []
    for final in range(len(shapeX), 1, -1): 
        for initial in range(final - 1, 1, -1):
            if (
                ((desiredSlopeX - (desiredSlopeX * tolerance)) < shapeX[final] - shapeX[initial] < (desiredSlopeX + (desiredSlopeX * tolerance))) and 
                ((desiredSlopeY - (desiredSlopeY * tolerance)) < shapeY[final] - shapeY[initial] < (desiredSlopeY + (desiredSlopeY * tolerance))) and 
                ((desiredSlopeZ - (desiredSlopeZ * tolerance)) < shapeZ[final] - shapeZ[initial] < (desiredSlopeZ + (desiredSlopeZ * tolerance)))
            ):
                foundSlopeIndices.append(final)
    if len(foundSlopeIndices) == 0:
        previousSlopeIndex = 0
        return 0 
    else: 
        foundSlopeIndices = list(set(foundSlopeIndices))
        newIndex = min(range(len(foundSlopeIndices)),key = lambda i: abs(foundSlopeIndices[i] - previousSlopeIndex)) 
        previousSlopeIndex = newIndex
        return newIndex

def translateRealToGraph(startingIndex: int): 
    global shapeX
    global shapeY
    global shapeZ
    shiftX = []
    shiftY = []
    shiftZ = []
    translateX = shapeX[startingIndex]
    translateY = shapeY[startingIndex]
    translateZ = shapeZ[startingIndex]
    for i in range(startingIndex, len(shapeX)):
        shiftX.append(shapeX[i] - translateX)
        shiftY.append(shapeY[i] - translateY)
        shiftZ.append(shapeZ[i] - translateZ)
    return shiftX, shiftY, shiftZ

def graphNeedle():
    slopeX = 0
    slopeY = 0
    slopeZ = 0 
    return translateRealToGraph(needleIndexInsideGel(slopeX, slopeY, slopeZ))

#method only to be run once 
def graphStart(theFile, offsetX, offsetY, offsetZ):
    global graphUtil
    figure = pyplot.figure()
    graphUtil.append(figure)
    axes = mplot3d.Axes3d(figure)
    graphUtil.append(axes)
    if theFile != None and os.path.exists(theFile):
        mesh = mesh.Mesh.from_file(theFile)
    #Compute the vector offset here instead of during normal graph method 
        mesh.x += offsetX
        mesh.y += offsetY
        mesh.z += offsetZ
        graphUtil.append(mesh)

def graph(offsetX, offsetY, offsetZ):
    global shapeX
    global shapeY
    global shapeZ 
    global graphUtil

    graphList = []
    #Possibily create a timer in order to limit frame rate
    
    for i in range(0, len(shapeX)):
        shapeX[i] = shapeX[i] - offsetX
        #if box is lower than fbgs then add 
        shapeY[i] = shapeY[i] + offsetY
        if shapeZ[i] <= 2.5 and shapeZ[i] >= -2.5:
            if shapeY[i] <= 5 and shapeY[i] >= 0:
                if shapeX[i] <= 5 and shapeX[i] >= 0:
                    shapeZ[i] = shapeZ[i] + 2.5 
                    graphList.append([shapeX[i], shapeY[i], shapeZ[i]])
                else:
                    continue 
            else:
                continue 
        else:
            continue  




def angles():
    global shapeX
    global shapeY
    global shapeZ
    OLD = -12 
    NEW = -1 
    x_old=float(shapeX[OLD])
    y_old=float(shapeY[OLD])
    z_old=float(shapeZ[OLD])
    x=float(shapeX[NEW])
    y=float(shapeY[NEW])
    z=float(shapeZ[NEW])

    unit=math.sqrt(((x-x_old)**2)+((y-y_old)**2)+((z-z_old)**2))
    unitXY=math.sqrt(((x-x_old)**2)+((y-y_old)**2))
    unitXZ=math.sqrt(((x-x_old)**2)+((z-z_old)**2))
    angleWX = math.asin((y-y_old)/unitXY) #math.atan((y-y_old) / (x-x_old))
    # rotationAboutZ = math.atan2(x-x_old, y-y_old) 
    # math.asin((z-z_old)/unitXZ)
    angleWZ= math.asin((z-z_old)/unit)
    # rotationAboutZ = math.atan2((y-y_old),(x-x_old))
    # rotationAboutY= math.atan2((z-z_old),(x-x_old)) 
    #time.sleep(2)
    roll=0
    pitch=angleWZ
    yaw=-angleWX 
    #print("Pitch: " + str(pitch)) 
    #
    # print("Yaw: " + str(yaw))
    angles = [roll, pitch, yaw] 
    #r = rotMatrix(angles, )
    return angles  

def setX(x):
    global shapeX
    shapeX = x 

def setY(y):
    global shapeY
    shapeY = y

def setZ(z): 
    global shapeZ
    shapeZ = z 

def getXYZ():
    global shapeX
    global shapeY
    global shapeZ
    return shapeX, shapeY, shapeZ

def parseData(): 
    global sock 
    global previousSlopeIndex
    global startX
    global startY
    global startZ
    global endX
    global endY
    global endZ
    try:
        while True:
            try:
                data = sock.recv(50000)
                data = str(data)
                data = data.split('\\t')
                #removes header from list 
                if len(data) > 100:
                    data[len(data) - 1] = data[len(data) - 1].replace('\\r\\n', '')
                    data[len(data) - 1] = data[len(data) - 1][0:len(data[len(data) - 1]) - 1]
                    data[len(data) - 1] = float(data[len(data) - 1])
                    for indice in range(0, len(data)):
                        if str(data[indice]).find('Shape x') > -1:
                            startX = indice + 2 
                        if str(data[indice]).find('Shape y') > -1:
                            startY = indice + 2
                            endX = indice   
                        if str(data[indice]).find('Shape z') > -1:
                            startZ = indice + 2 
                            endY = indice 
                    diffX = endX - startX
                    previousSlopeIndex = diffX
                    endZ = startZ + diffX
                    break 
            except Exception:
                pass         
    except Exception:
        pass 

def tcp():
    # Create a TCP/IP socket
    global shapeX
    global shapeY
    global shapeZ
    global curvatureAngle
    global Curvature
    global sock
    global counter 
    global startX
    global startY
    global startZ
    global endX
    global endY
    global endZ
    try:
        while True: 
            try: 
                index = 0
                data = sock.recv(50000)
                data = str(data)
                data = data.split('\\t')
                #removes header from list 
                if len(data) > 10:
                    data[len(data) - 1] = data[len(data) - 1].replace('\\r\\n', '')
                    data[len(data) - 1] = data[len(data) - 1][0:len(data[len(data) - 1]) - 1]
                    data[len(data) - 1] = float(data[len(data) - 1])
                    shapeX = data[startX:endX]                     
                    shapeY = data[startY:endY]
                    shapeZ = data[startZ:endZ]
                    #Shape Z Index 808
                    #Shape Y Index 519 
                    #Shape X Index 230 
                    #Curvature [1/cm] 188 
                    #Curvature angle 209
                    setX(shapeX)
                    setY(shapeY) 
                    setZ(shapeZ)
                    angleX, angleY, angleZ = angles()
                    index += 1 
                    return angleX, angleY, angleZ
            except ValueError: 
                traceback.print_exc()
                print("Error Converting String to Float")
                continue
            except TypeError: 
                traceback.print_exc()
                print("String Detected")
                continue 
            except Exception: 
                traceback.print_exc()
                continue 

    except Exception: 
        print("ERROR")
        traceback.print_exc()

def closeTCP():
    global sock 
    sock.close()
    print("Closing Socket")

#ORDER IS XYZ
def rotvec2rpy(rotvec, degreesFlag=True):
    r = R.from_rotvec(list(rotvec)) 
    return list(r.as_euler('xyz', degrees=bool(degreesFlag)))

def rpy2rotvec(rpy, degreesFlag=True): 
    r = R.from_euler('zyx', rpy, degrees=bool(degreesFlag)) 
    return list(r.as_rotvec())

def rotMatrix(angleList,axisList):
    R = np.identity(3)

    for angle,axis in zip(angleList,axisList):
        c = np.cos(angle)
        s = np.sin(angle)

        if axis == 'Rx':
            rotMat = np.array([[1,0,0],[0,c,-s],[0,s,c]])

        elif axis == 'Ry':
            rotMat = np.array([[c,0,s],[0,1,0],[-s,0,c]])
        
        else:
            rotMat = np.array([[c,-s,0],[s,c,0],[0,0,1]])
        
        R = np.matmul(R,rotMat)

    return R

def averageAngle(size): 
    angleX = 0
    angleY = 0
    angleZ = 0
    tempX = 0
    tempY = 0
    tempZ = 0
    for i in range(0, size): 
        tempX, tempY, tempZ = tcp()
        angleX += tempX 
        angleY += tempY
        angleZ += tempZ
    angleX = angleX / size
    angleY = angleY / size
    angleZ = angleZ / size
    return angleX, angleY, angleZ

startup()
parseData()
while True:
    angleX, angleY, angleZ = averageAngle(10) 

    print(math.degrees(angleX), math.degrees(angleY), math.degrees(angleZ))
# pose = [0,0,0,0,0,0]
# pose_wrt_base = UR_Move_RTDE.rtde__c.poseTrans(UR_Move_RTDE.rtde__r.getActualTCPPose(), pose)
# print(str(pose_wrt_base))
# pose = [0,0,0,0,0,0]
# pose_wrt_base = UR_Move_RTDE.rtde__c.poseTrans(pose, UR_Move_RTDE.rtde__r.getActualTCPPose())
# print(str(pose_wrt_base))
# print(str(UR_Move_RTDE.rtde__r.getActualTCPPose()))
# print(str(rotvec2rpy(pose_wrt_base[3:], degreesFlag=False)))
#UR_Move_RTDE.rtde__c.moveL(pose_wrt_base)
#BIG COMMENT: YO FUTURE JAMES, THE DANG ROBOT IS IN METERS.  
#print(str( UR_Move_RTDE.rtde__r.getActualTCPPose()))
#UR_Move_RTDE.rtde__c.moveL(pose_wrt_base)
# UR_Move_RTDE.rtde__c.moveL(pose_wrt_base)
#UR_Move_RTDE.quit()

