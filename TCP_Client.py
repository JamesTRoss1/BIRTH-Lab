
from operator import contains
import socket
import sys
import time
import traceback
import re
from tracemalloc import start
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import offsetbox, pyplot, cm
from matplotlib.colors import LightSource
import os
import pathlib
#UR_Move_RTDE.startup()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Curvature = []
curvatureAngle = []
shapeX = []
counter = 0
shapeY = []
graphUtil = []
shapeZ = []
figure = None
axes = None
meshes = None
startX = 0
startY = 0
startZ = 0
endX = 0
endY = 0
endZ = 0
# Connect the socket to the port where the server is listening
def startup():
    server_address = ('127.0.0.1', 5000)
    print('connecting to {} port {}'.format(*server_address))
    sock.connect(server_address)
    print("connected")
    parseData()

#method only to be run once
def graphStart(theFile = None, offsetX = 0, offsetY = 0, offsetZ = 0):
    global figure
    global axes
    global meshes
    figure = pyplot.figure()
    axes = figure.add_subplot(111, projection='3d')
    print(str(os.path.exists(theFile)))
    if theFile != None and os.path.exists(theFile):
        meshes = mesh.Mesh.from_file(theFile)
        #THE_MESH = mplot3d.art3d.Poly3DCollection(meshes.vectors, alpha=.6, fc="r", ec="w")
            #THE_MESH.set_facecolor("#000000")
            #THE_MESH.set_edgecolor("red")
        #axes.add_collection3d(THE_MESH)
        print(str(meshes.vectors))
    #Compute the vector offset here instead of during normal graph method

def graph(offsetX = 0, offsetY = 0, offsetZ = 0, meshBool = True):
    try:
        global shapeX
        global shapeY
        global shapeZ
        global figure
        global meshes
        global axes
        points = []
        #Possibily create a timer in order to limit frame rate
        #Remove points instead of clearing graph; unable to rotate by clearing graph
        #axes = figure.add_subplot(111, projection='3d')
        start = time.time()

        if meshBool:
            #ls = LightSource(270, 45)
# To use a custom hillshading mode, override the built-in shading and pass
# in the rgb colors of the shaded surface calculated from "shade".
            #print(str(dir(meshes)))
            pass
            #rgb = ls.shade(data=meshes.z, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
            #Keep stl under 10000 triangles

            THE_MESH = mplot3d.art3d.Poly3DCollection(meshes.vectors, alpha=.3, fc="r", ec="r")
            #THE_MESH.set_facecolor("#000000")
            #THE_MESH.set_edgecolor("red")
            axes.add_collection3d(THE_MESH)
            #frame = axes.plot_surface(meshes.x, meshes.y, meshes.z)
            ##frame.set_edgecolor("r")
            #frame.set_facecolor("r")
            #frame.set_alpha(.03)
            #axes.plot3D(meshes.vectors, 'red')
            #print(str(meshes.vectors))
        ##print("Plotting")
        #print("ShapeX ", str(shapeX))
        #print("ShapeY ", str(shapeY))
        #print("ShapeZ ", str(shapeZ))
        if True:
            for i in range(len(shapeX) - 1, 0, -1):
                shapeX[i] = shapeX[i] - offsetX
                shapeY[i] = shapeY[i] + offsetY
                if shapeZ[i] <= 2.5 and shapeZ[i] >= -2.5:
                    if shapeY[i] <= 5 and shapeY[i] >= 0:
                        if shapeX[i] <= 5 and shapeX[i] >= 0:
                            shapeZ[i] = shapeZ[i] + 2.5
                            points.append(axes.plot(shapeX[i], shapeY[i], shapeZ[i]))
                        else:
                            break
                    else:
                        break
                else:
                    break
        axes.set_xlabel("x")
        axes.set_ylabel("y")
        axes.set_zlabel("z")
        axes.set_autoscale_on(True)
        #axes.set(xlim=(-300, 300), ylim=(-300, 300), zlim=(-300, 300))
        pyplot.show(block=False)
        pyplot.pause(.01)
        axes.clear()
        end = time.time()
        print("TIME: " + str(end - start))
    except Exception as e:
        print(str(e))

def angles1():
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


def angles():
    global shapeX
    global shapeY
    global shapeZ
    #print(shapeX)
    OLD = -12
    NEW = -1
    # rotationMat = [[-10.8335,10.749,6.204],[3.,3.29,1.11],[0.9,2.35,-2.5]]
    # rotationMat = [[-0.99,0,.04],[0.05,-0.22,.97],[0,0.99,0.1]]
    rotationMat = np.transpose([[0.98,0,.194],[-0.15,-0.98,-.097],[0.19,0.12,-0.965]])

    x_old=float(shapeX[OLD])
    y_old=float(shapeY[OLD])
    z_old=float(shapeZ[OLD])
    oldCord= [[x_old],[y_old],[z_old]]
    NewOld = np.matmul(rotationMat,oldCord)
    x=float(shapeX[NEW])
    y=float(shapeY[NEW])
    z=float(shapeZ[NEW])

    newCord= [[x],[y],[z]]
    NewNew = np.matmul(rotationMat,newCord)
    #print(x,y,z,"new",NewNew[0],NewNew[1],NewNew[2] )

    # unit=math.sqrt(((x-x_old)**2)+((y-y_old)**2)+((z-z_old)**2))
    # unitXY=math.sqrt(((x-x_old)**2)+((y-y_old)**2))
    # unitXZ=math.sqrt(((x-x_old)**2)+((z-z_old)**2))
    # angleWX = math.asin((y-y_old)/unitXY) #math.atan((y-y_old) / (x-x_old))
    # # rotationAboutZ = math.atan2(x-x_old, y-y_old)
    # # math.asin((z-z_old)/unitXZ)
    # angleWZ= math.asin((z-z_old)/unit)
    unit=math.sqrt(((NewNew[0] - NewOld[0])**2)+((NewNew[1]-NewOld[1])**2)+((NewNew[2]-NewOld[2])**2))
    unitXY=math.sqrt(((NewNew[0] - NewOld[0])**2)+((NewNew[1]-NewOld[1])**2))
    unitXZ=math.sqrt(((NewNew[0] - NewOld[0])**2)+((NewNew[2]-NewOld[2])**2))
    angleWX = math.asin((NewNew[1]-NewOld[1])/unitXY) #math.atan((y-y_old) / (x-x_old))
    angleWZ= math.asin((NewNew[1]-NewOld[1])/unit)

    # rotationAboutZ = math.atan2(x-x_old, y-y_old)
    # math.asin((z-z_old)/unitXZ)
    # unit=math.sqrt(((x-x_old)**2)+((y-y_old)**2)+((z-z_old)**2))
    # unitXY=math.sqrt(((x-x_old)**2)+((y-y_old)**2))
    # unitXZ=math.sqrt(((x-x_old)**2)+((z-z_old)**2))
    # angleWX = math.asin((y-y_old)/unitXY) #math.atan((y-y_old) / (x-x_old))
    # # rotationAboutZ = math.atan2(x-x_old, y-y_old)
    # # math.asin((z-z_old)/unitXZ)
    # angleWZ= math.asin((z-z_old)/unit)
    # rotationAboutZ = math.atan2((y-y_old),(x-x_old))
    # rotationAboutY= math.atan2((z-z_old),(x-x_old))
    # angleWZ= math.asin((NewNew[2]-NewOld[2])/unit)
    # rotationAboutZ = math.atan2((y-y_old),(x-x_old))
    # rotationAboutY= math.atan2((z-z_old),(x-x_old))
    #time.sleep(2)
    roll=0
    pitch = angleWZ
    yaw = -angleWX
    #pitch=angleWZ)
    #yaw=math.degrees(-angleWX)
    #print("Pitch: " + str(pitch))
    #
    # print("Yaw: " + str(yaw))
    angles = [roll, pitch, yaw]
    #r = rotMatrix(angles, )
    return angles
    #return NewOld


def checkPos():
    global shapeX
    global shapeY
    global shapeZ
    #print(shapeX)
    NEW = -1
    x=float(shapeX[NEW])
    y=float(shapeY[NEW])
    z=float(shapeZ[NEW])
    print("POS (x,y,z): ", str(x), str(y), str(z))

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
    global startX
    global startY
    global startZ
    global endX
    global endY
    global endZ
    try:
        while True:
            try:
                data = sock.recv(50000000)
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
                    angleX, angleY, angleZ = angles1()
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
def checkAngles():
    while True:
        angleX, angleY, angleZ = tcp()
        #NewOld = tcp()
        # print("Angles: ", math.degrees(angleX), math.degrees(angleY), math.degrees(angleZ))
        print("Angles: ", math.degrees(angleX), math.degrees(angleY), math.degrees(angleZ))
        # print("Pos:",NewOld)

        #print(shapeZ[-12])
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

#startup()
#parseData()
#while True:
#    print(str(tcp()))

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
