
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
    for final in range(len(shapeX) - 1, 1, -1): 
        for initial in range(final - 1, 0, -1):
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
    tcp()
    return translateRealToGraph(needleIndexInsideGel(slopeX, slopeY, slopeZ))

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

def gradientInit():
    global shapeX 
    global shapeY
    global shapeZ 
    global Oxx 
    global indX
    global indX2

    newX = shapeX[0: len(shapeX) - 1]
    newY = shapeY[0: len(shapeY) - 1]
    newZ = shapeZ[0: len(shapeZ) - 1]

    #dx = np.gradient(newX) l-shape 
    dx = np.gradient(np.gradient(newX)) #s-shape ??? 
    plt.plot(dx)
    getpoint = plt.ginput(2, 0)   # (x1, y1), (x2, y2) 

    indX = np.array(np.floor(getpoint))
    Oxx = int(max(range(len(dx)), key = lambda i: dx[i] + indX[0,0]))   #Possible solution for s shape 
    """
    line = np.arange(0, indX[1,0] - indX[0, 0])
    start = int(indX[0, 0])
    end = int(indX[1, 0])
    f = np.polyfit(line, dx[start : end], 4)
    y1 = np.polyval(f,line)
    dy1 = np.gradient(np.gradient(y1))
    plt.plot(dy1)
    Oxx = int(min(range(len(dy1)),key = lambda i: dy1[i]) + indX[0, 0])
    """
    print(str("Oxx: " + str(Oxx)))
    OriginX = newX[Oxx]
    OriginY = newY[Oxx]
    OriginZ = newZ[Oxx]

    plt.plot(dx)
    getpoint = plt.ginput(2, 0)

    indX2 = np.array(np.floor(getpoint))

    AxisXX = np.array([[newX[int(indX2[0,0])] - newX[int(indX[0,0])]], [newY[int(indX2[0,0])] - newY[int(indX[0,0])]], [newZ[int(indX2[0,0])] - newZ[int(indX[0,0])]]])

    AxisYY = np.array([[newX[int(indX2[1,0])] - newX[int(indX[1,0])]], [newY[int(indX2[1,0])] - newY[int(indX[1,0])]], [newZ[int(indX2[1,0])] - newZ[int(indX[1,0])]]])
    AxisX = AxisXX / np.linalg.norm(AxisXX)
    AxisY = AxisYY / np.linalg.norm(AxisYY)

    AxisZ = np.cross(AxisX,AxisY, axis=0)
    AxisY = np.cross(AxisX,AxisZ, axis=0)

def needleByGradient():
    global shapeX 
    global shapeY
    global shapeZ 
    global indX
    global indX2
    global Oxx
    
    newX = shapeX[0: len(shapeX) - 1]
    newY = shapeY[0: len(shapeY) - 1]
    newZ = shapeZ[0: len(shapeZ) - 1]

    #dx = np.gradient(newX) l-shape 
    dx = np.gradient(np.gradient(newX)) #s-shape ??? 

    line = np.arange(0, indX[1,0] - indX[0, 0])
    start = int(indX[0, 0])
    end = int(indX[1, 0])
    f = np.polyfit(line, dx[start : end], 4)
    y1 = np.polyval(f,line)
    dy1 = np.gradient(np.gradient(y1))
    previousOxx = Oxx 
    Oxx = int(min(range(len(dy1)),key = lambda i: dy1[i]) + indX[0, 0])  #changed to max for s shape 
    changeOxx = Oxx - previousOxx

    OriginX = newX[Oxx]
    OriginY = newY[Oxx]
    OriginZ = newZ[Oxx]

    AxisXX = np.array([[newX[int(indX2[0,0])] - newX[int(indX[0,0])]], [newY[int(indX2[0,0])] - newY[int(indX[0,0])]], [newZ[int(indX2[0,0])] - newZ[int(indX[0,0])]]])

    AxisYY = np.array([[newX[int(indX2[1,0])] - newX[int(indX[1,0])]], [newY[int(indX2[1,0])] - newY[int(indX[1,0])]], [newZ[int(indX2[1,0])] - newZ[int(indX[1,0])]]])
    AxisX = AxisXX / np.linalg.norm(AxisXX)
    AxisY = AxisYY / np.linalg.norm(AxisYY)

    AxisZ = np.cross(AxisX,AxisY, axis=0)
    AxisY = np.cross(AxisX,AxisZ, axis=0)

    indX[0, 0] = indX[0,0] + float(changeOxx)
    indX[1, 0] = indX[1,0] + float(changeOxx)
    indX2[0,0] = indX2[0,0] + float(changeOxx)
    indX2[1,0] = indX2[1,0] + float(changeOxx)

    #Relative point calculation 

    halfTube = 5  #half tube length in cm

    relIndex = min(range(int(indX2[0,0]), int(indX2[1,0]) + 1), key = lambda i: math.fabs(shapeX[i] - (shapeX[Oxx] + halfTube))) 
    relativePoints = []
    for point in range(relIndex, len(shapeX)):
        relativePoints.append([shapeX[point] - shapeX[relIndex], shapeY[point] - shapeY[relIndex], shapeZ[point] - shapeZ[relIndex]])

    relativePoints = np.array(relativePoints)
    theta = math.atan2(AxisZ[2], AxisZ[1])
    ax = AxisX
    ay = Rx(theta) * AxisY 
    az = Rx(theta) * AxisZ
    RR = np.array([ax, ay, az])
    dotPoints = np.dot(relativePoints, RR)
    print(str(Oxx))
    return dotPoints.tolist()

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



