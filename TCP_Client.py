
import socket
import sys
import time 
import traceback
import re
import math

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Curvature = []
curvatureAngle = []
shapeX = []
shapeY = []
shapeZ = []
# Connect the socket to the port where the server is listening
server_address = ('127.0.0.1', 5000)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)
print("connected")

def angles():
    global shapeX
    global shapeY
    global shapeZ
    x_old=float(shapeX[-4])
    y_old=float(shapeY[-4])
    # print(shapeX)
    # print()
    # print(shapeY)
    # print()
    # print(shapeZ)
    # print()
    z_old=float(shapeZ[-4])
    x=float(shapeX[-1])
    y=float(shapeY[-1])
    z=float(shapeZ[-1])
    # print(str(x))
    # print(str(y))
    # print(str(z))
    # print(str(x_old))
    # print(str(y_old))
    # print(str(z_old))
    unit=math.sqrt(((x-x_old)**2)+((y-y_old)**2)+((z-z_old)**2))
    angleX=math.acos((x-x_old)/unit)
    angleY=math.acos((y-y_old)/unit)
    angleZ=math.acos((z-z_old)/unit)


    return angleX,angleY,angleZ


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

def tcp():
    # Create a TCP/IP socket
    global shapeX
    global shapeY
    global shapeZ
    global curvatureAngle
    global Curvature
    global sock

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
            Curvature = data[189:209]
            curvatureAngle = data[210:230]
            shapeX = data[231:519]
            shapeY = data[520:808]
            shapeZ = data[809:]
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
    except Exception: 
        traceback.print_exc()

def closeTCP():
    global sock 
    sock.close()
    print("Closing Socket")
