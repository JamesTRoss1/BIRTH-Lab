import numpy as np
import vispy
from vispy import app, gloo, geometry, io, visuals, scene
vispy.use('PyQt5')
from vispy.app.canvas import KeyEvent
from vispy.geometry import create_sphere
import math
import copy
from vispy.scene.visuals import Mesh
from vispy.visuals.transforms import (STTransform, MatrixTransform,
                                  ChainTransform)
from vispy.util.quaternion import Quaternion
from vispy.scene.events import SceneMouseEvent
from vispy.util.keys import Key
import trimesh
#import TCP_Client_Test
#import xboxVel #uncomment when using xbox controller
from vispy.util.event import *
import vispy.scene
from vispy.scene.visuals import XYZAxis, Tube
from vispy.scene.widgets.axis import AxisWidget
import time
import os
from vispy.visuals.filters.color import Alpha, ColorFilter
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
import pythonping
from functools import partial
import pathlib
import subprocess
from vispy.app.qt import QtSceneCanvas
from vispy.visuals.line_plot import LinePlotVisual
from vispy.visuals.filters.mesh import WireframeFilter, ShadingFilter
import matplotlib
matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


LINE = None
ui = None
startTime = 0
testNum = 0
xyz = None
view = None
initialPose = None
initialPoseBool = False
Plot3D = scene.visuals.create_visual_node(visuals.LinePlotVisual)
canvas1 = None
counter = 0
canvas2 = None
shading_filter = None
wireframe_filter = None
file = ""
POS = None
sumP = 0
axes = None
TUBE = None
sumY = 0
xboxPoints = []
SCALE = None
axes3 = None

class MyCanvas(vispy.app.qt.QtSceneCanvas):

    def __init__(self, firstView = True):
        global file
        global view
        global LINE
        global Plot3D
        global xyz
        global wireframe_filter
        global initialPose
        self.firstView = firstView
        print(str(id(self)) + " : " + str(firstView))
        if not firstView:
            vispy.app.qt.QtSceneCanvas.__init__(self, keys='interactive', size=(1080, 720), bgcolor=(0,0,0,0))
        else:
            vispy.app.qt.QtSceneCanvas.__init__(self, keys=None, size=(1080, 720), bgcolor=(0,0,0,0))
        self.unfreeze()
        print("ID: " + str(id(self)))
        self.meshes = []
        self.filters = []
        self.view = self.central_widget.add_view()
        self.view.camera = 'fly'
        self.view.camera.fov = 90
        self.view.camera.scale_factor = 1.0
        self.view.camera.zoom_factor = -1
        self.view.camera.center = (0,0,0)
        #x:90-forward is z; y:90- forward x; z:90-forward y
        self.view.camera.rotation1 = Quaternion.create_from_euler_angles(90,90,0, True)
        if not firstView:
            self.view.camera.interactive = True
            self.view.camera.auto_roll = True
        else:
            self.view.camera.interactive = True
            self.view.camera.auto_roll = False
        varMesh = trimesh.load(file)
        print(str(varMesh.vertices))
        print(str(varMesh.vertices))
        self.mesh = None
        if not firstView:
            self.connect(self.on_key_press)
        mdata = geometry.MeshData(varMesh.vertices, varMesh.faces)
        print(str(varMesh.vertices))
        if firstView:
            self.mesh = Mesh(meshdata=mdata, shading=None, color=(1,0,0,.6), parent=self.view.scene)
            self.shading_filter = ShadingFilter(shading='flat', diffuse_light=(1,1,1,.7), ambient_light = (1, 1, 1, .4), specular_light = (1,1,1,.5), shininess=30)
            self.mesh.attach(self.shading_filter)

        if not firstView:
            self.mesh = Mesh(meshdata=mdata, shading=None, color=(1,1,1,1), parent=self.view.scene)
            self.wireframe_filter = WireframeFilter(wireframe_only=True, color='white', width=2)
            self.mesh.attach(self.wireframe_filter)
            #self.mesh.attach(ColorFilter(filter = (.2, .2, 1, 1)))
            #self.mesh.attach(Alpha(.7))
            self.shading_filter = ShadingFilter(shading='flat', diffuse_light=(1,1,1,.9))
            self.mesh.attach(self.shading_filter)

        self.attach_headlight()
        self.timer = vispy.app.Timer(connect = self.wait)
        self.timer.start(0.1)
        finalArr = list(self.view.camera.center)
        initialPose = list(self.view.camera.center)

    def attach_headlight(self):
        light_dir = (0, 1, 0, 0)
        self.shading_filter.light_dir = light_dir[:3]
        initial_light_dir = self.view.camera.transform.imap(light_dir)

        @self.view.scene.transform.changed.connect
        def on_transform_change(event):
            transform = self.view.camera.transform
            self.shading_filter.light_dir = transform.map(initial_light_dir)[:3]

    def wait(self, event):
        global canvas1
        global counter
        global axes
        global POS
        global SCALE
        global TUBE
        global axes3
        #change scale factor if you need the fly camera to move at a specific speed
        counter = counter + 1

        new_pos = None
        if counter < 5000000:
            print(str(canvas1.view.camera.zoom_factor))
            pose = _get_position(1, 0, 0, .1)
            canvas1.view.camera.center = pose
            axes_pos2 = _get_position(2, 0, 0, .1)
            pos2 = list(axes_pos2)
            xpos2 = copy.copy(pos2)
            xpos2[0] = xpos2[0] + .5
            ypos2 = copy.copy(pos2)
            ypos2[1] = ypos2[1] + .5
            zpos2 = copy.copy(pos2)
            zpos2[2] = zpos2[2] + .5
            new_pos = np.array([
            pos2, xpos2, pos2, ypos2, pos2, zpos2
            ])

            #Position of the 1st person camera on the 3rd person canvas
            pos3 = list(canvas1.view.camera.center)
            xpos3 = copy.copy(pos3)
            xpos3[0] = xpos3[0] + 1
            ypos3 = copy.copy(pos3)
            ypos3[1] = ypos3[1] + 1
            zpos3 = copy.copy(pos3)
            zpos3[2] = zpos3[2] + 1
            new_pos2 = np.array([
            pos3, xpos3, pos3, ypos3, pos3, zpos3
            ])

            if hasattr(axes, 'parent'):
                axes.parent = None
                axes = XYZAxis(parent=canvas1.view.scene, pos=new_pos)
                axes3.parent = None
                axes3 = XYZAxis(parent=canvas2.view.scene, pos=new_pos2)
            else:
                axes = XYZAxis(parent=canvas1.view.scene)
                axes3 = XYZAxis(parent=canvas2.view.scene)


            farCenter = _get_position(200, 0, 0, .1)
            startTube = _get_position(0, 1.5, -1.5, .1)
            _changeX = (farCenter[0] - startTube[0]) * .05
            _changeY = (farCenter[1] - startTube[1]) * .05
            _changeZ = (farCenter[2] - startTube[2]) * .05
            endTube = ((_changeX) + startTube[0], (_changeY) + startTube[1], (_changeZ) + startTube[2])
            tubeArr = np.array([
            startTube, endTube
            ])
            print("TUBE ARR: " + str(tubeArr))
            print("CENTER: " + str(canvas1.view.camera.center))

            #TUBE IS ABOUT 1 CENTIMETER IN LENGTH
            if hasattr(TUBE, 'parent'):
                TUBE.parent = None
                TUBE = Tube(points=tubeArr, radius=.01, color= "blue", parent=canvas1.view.scene)

            else:
                TUBE = Tube(points=tubeArr, radius=.01, color= "blue", parent=canvas1.view.scene)

            #keyEvent = KeyEvent("key_press", key=Key('W'), text='W')
            #canvas1.view.camera.viewbox_key_event(keyEvent)
            #scalePoints = _get_position(2, 1.0, -1.5 ,.1)
            #yPos2 = _get_position(2, 2, -1.5, .1)
            #scale_points = np.array([
            #scalePoints, scalePoints, scalePoints, yPos2, scalePoints, scalePoints
            #])
            #if hasattr(SCALE, 'parent'):
            #    SCALE.parent = None
            #    SCALE = Plot3D(scale_points, width=1, color=(1,1,1,1), edge_color=(1, 1, 1, 1), face_color=(1, 1, 1, 1), parent=canvas1.view.scene)
            #else:
            #    SCALE = Plot3D(parent=canvas1.view.scene)


        else:
            pass
            #keyEvent = KeyEvent("key_release", key=Key('W'), text='W')
            #canvas1.view.camera.viewbox_key_event(keyEvent)
            #canvas1.view.camera.view_changed()

    def on_key_press(self, key):
        global initialPoseBool
        global view
        global initialRotation
        global initialNeedleRotation
        global initialPose
        global canvas1
        global canvas2
        global wireframe_filter
        print("KEY: " + str(key.text))
        if ord(key.text) == 13:
            print("CENTER: " + str(self.view.camera.center))
            print("Rotation: " + str(self.view.camera.rotation1))
        if ord(key.text) == 13 and initialPoseBool == False:
            canvas1.view.camera.center = canvas2.view.camera.center
            canvas1.view.camera.rotation1 = canvas2.view.camera.rotation1
            canvas1.view.camera.view_changed()
            initialPoseBool = True
            initialPose = list(canvas1.view.camera.center)
            initialRotation = canvas2.view.camera.rotation1
            #TCP_Client.startup()
            #initialNeedleRotation = TCP_Client.tcp()
            initialNeedleRotation = (0,0,0) #COMMENT OUT WHEN USING TCP_cLIENT MODULE
            canvas1.timer.stop()
            canvas1.timer = vispy.app.Timer(connect = canvas1.line)
            canvas1.timer.start(0.05)
            self.timer.stop()
            print("ID FROM THE ENTER: " + str(id(self)))
        #if key.text == ".":
        #    canvas1.view.camera.zoom_factor = canvas1.view.camera.zoom_factor + 1
        #if key.text == ",":
        #    if canvas1.view.camera.zoom_factor - 1 > 0:
        #        canvas1.view.camera.zoom_factor = canvas1.view.camera.zoom_factor - 1
        if key.text == "+" or key.text == "=":
            self.view.camera.scale_factor = self.view.camera.scale_factor + 1
        if key.text == "-":
            if self.view.camera.scale_factor - 1 > 0:
                self.view.camera.scale_factor = self.view.camera.scale_factor - 1
        if key.text == ";":
            wireframe_filter.enabled = not wireframe_filter.enabled
            canvas2.mesh.color = (1,0,0,.6)
            canvas2.update()
    def needleStart(self):
        print("HERE")

    def pos(self, event):
        global view
        print(str(self.view.camera.center))

#CANVAS1 CALLS THIS METHOD
    def line(self, event):
        global startTime
        global view
        global xyz
        global testNum
        global LINE
        global initialPose
        global Plot3D
        global initialRotation
        global initialNeedleRotation
        global origin
        global counter
        global canvas1
        global canvas2
        global sumP
        global sumY
        global xboxPoints

        xPoints = []
        yPoints = []
        zPoints = []
        #TCP_Client_Test.testTcp(counter // 100)
        #points, origin = TCP_Client_Test.needleBySlope() #first 3 elements in returned list is the 3d point representation of the needle tip; last is new origin
        #rpy_ = TCP_Client.tcp()
        #points = TCP_Client.needleByGradient()
        points = xboxVel.joyStickNeedleControl(0,0,0,0,0,0)
        rpy_ = (0, points[4] * .1, points[5] * .1)
        changeP = rpy_[1]
        changeY = rpy_[2]
        #
        #changeP = rpy_[1] - initialNeedleRotation[1]
        #changeY = rpy_[2] - initialNeedleRotation[2]
            #rpy2quat returns a quaternion in x,y,z,w format
            #Vispy Quaternion constructor is in w,x,y,z format
            #Might need to call Quaternion.conjugate if the rpy2quat is opposite
        #rx has object appear to move in a counterclockwise direction
        #ry is side to side; positive toward right
        #rz is up and down; positive toward down
        #x forward is positive
        #y to the left is positive
        #z upwards is positive
        sumP = sumP + changeP
        sumY = sumY + changeY
        # matrix = MatrixRotation(0, 180 - sumY, 180 + sumP)
        matrix = MatrixRotation(0, sumY, 0)

        needleQuat = Quaternion.create_from_euler_angles(0, sumP, sumY, True)
        #newQuatArr = needleQuat
        newQuatArr = needleQuat * initialRotation #Quat multiplication necessary to add two together
        #initialRotation = rpy
        thePoints = np.array([points[2], 0, 0]).reshape(3,1)
        points = np.dot(matrix, thePoints)
        print("POINTS")
        print(str(points))
        print("DA POINTS")
        print(str(thePoints))
        print("MATRIX")
        print(str(matrix))
        xPoints.append(points[0] * .1)
        yPoints.append(points[1] * .1)
        zPoints.append(points[2] * .1)


        #XBOX DEMO
        #MAKE SURE THAT THE POINTS MATCH THE COORDINATE FRAME OF THE VISPY CANVAS
        #for point in range(0, len(points)):
        #    xPoints.append(points[point][0])
        #    yPoints.append(points[point][1])
        #    zPoints.append(points[point][2])
        length_ = len(xPoints) - 1
        self.view.camera.center = (xPoints[length_] + initialPose[0], yPoints[length_] + initialPose[1], zPoints[length_] + initialPose[2])
        initialPose = self.view.camera.center
        counter = counter + 1
        if counter % 10 == 0:
            if len(xboxPoints) > 10:
                xboxPoints.pop(0)
                xboxPoints.append(self.view.camera.center)
            else:
                xboxPoints.append(self.view.camera.center)

        self.view.camera.rotation1 = newQuatArr
        self.view.camera.view_changed()

        #CANVAS 2 LOGIC
        arr = xboxPoints
        #arr = []
        if hasattr(LINE, 'parent'):
            LINE.parent = None
        #for i in range(0, len(xPoints)):
        #    temp = [xPoints[i] + initialPose[0], yPoints[i] + initialPose[1], zPoints[i] + initialPose[2]]
        #    arr.append(temp)
        LINE = Plot3D(arr, width=3, color=(0,0,1,1), edge_color=(1, 1, 1, 1), symbol='o', face_color=(.2, .2, 1, 1), parent=canvas2.view.scene)

class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class Color(QWidget):
    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)

class Ui_MainWindow(QWidget):
    def setupUi(self, MainWindow):
        MainWindow.resize(1000, 1000)
        self.centralwidget = QWidget(MainWindow)
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QRect(250, 250, 93, 28))

		# For displaying confirmation message along with user's info.
        self.label = QLabel(self.centralwidget)
        self.label.setGeometry(QRect(0, 0, 500, 120))

		# Keeping the text of label empty initially.
        self.label.setText("")
        MainWindow.setCentralWidget(self.centralwidget)
        self.retranslateUi(MainWindow)
        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Robot-PC Interface"))
        self.pushButton.setText(_translate("MainWindow", "Begin Setup"))
        print(type(MainWindow))
        take_input = partial(self.takeinputs, MainWindow)
        self.pushButton.clicked.connect(take_input)

    #Items must be a list of strings
    def combo(self, MainWindow, items):
        print(str(type(MainWindow)))
        self.centralwidget = QComboBox(self)
        for item in items:
            self.centralwidget.addItem(item)
        self.centralwidget.move(50, 250)
        MainWindow.setCentralWidget(self.centralwidget)

    def takeinputs(self, MainWindow):
        global file
        ip, done1 = QInputDialog.getText(self, 'Input Dialog', 'Enter robot IP address:')
        offset = 0
        res = pythonping.ping(str(ip), timeout=.2)
        while int(res.packets_lost) != 0:
            ip, done1 = QInputDialog.getText(self, 'Input Dialog', 'No Response From IP Address. Try Again')
            res = pythonping.ping(str(ip), timeout=.2)
        #define offset in cm
        offset, done2 = QInputDialog.getText(self, 'Input Dialog', 'Enter needle offset (cm):')
        offset = float(offset)
        dicom_file , done3 = QFileDialog.getOpenFileName(None, "Choose Zip of DICOM Files or STL file for the surgery",
                                                "", "(*.stl);;(*.zip)")
        dicom_file = str(dicom_file)
        dcm_files = pathlib.Path(dicom_file)
        file = dcm_files
        #Future Idea: implememnt a combo box with button in order to provide user access to parameters for conversion file (dicom2stl.py)

        if dcm_files.suffix == ".zip":
            name = dcm_files.stem
            new_file = name + ".stl"
            command = "python dicom2stl.py -t tissue -o " + new_file + " " + dcm_files
            ret = subprocess.call(command, shell=True)  #convert the zip of dicom files and then save as same file with extension .stl
            #find file and create File IO wrapper for file


        #png, done4 = QFileDialog.getOpenFileName(None, "Choose JPG or PNG file for top-down view", "", "(*.png);;(*.jpg)")
        #png = str(png)
        self.label.setText("Robot IP Address: " + str(ip) + "\n" +
                            "Needle Offset: " + str(offset) + "\n" +
                            "Dicom or STL File: " + str(dicom_file) + "\n")
        self.label.adjustSize()
        self.pushButton1 = QPushButton(self.centralwidget)
        self.pushButton1.setText("Restart")
        self.pushButton.setText("Continue")
        self.pushButton.setGeometry(150, 250, 100, 100)
        self.pushButton1.setGeometry(400, 250, 100, 100)
        take_input = partial(self.takeinputs, MainWindow)
        self.pushButton1.clicked.connect(take_input)
        self.pushButton.clicked.disconnect()
        finish = partial(self.finished_input, ip = str(ip), offset = str(offset))
        self.pushButton.clicked.connect(finish)
        self.pushButton1.show()

    def finished_input(self, canvas, ip = "", offset = 0):
        global file
        global canvas1
        global canvas2
        self.pushButton.hide()
        self.pushButton1.hide()
        self.label.hide()
        canvas1 = MyCanvas(firstView=True)
        canvas2 = MyCanvas(firstView=False)
        lay = QGridLayout() #Choose gridlayout for vispy
        self.centralwidget.setLayout(lay)
        #lay.addWidget(Color('red'), 0, 0, 1, 3) # y, x, span along y, span along x
        lay.addWidget(Color('red'), 0, 0, 1, 4)
        lay.addWidget(canvas2, 1, 2, 3, 2)
        lay.addWidget(canvas1, 1, 0, 3, 2)
        self.pushButton.hide()


def MatrixRotation(roll, pitch, yaw):
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    yaw = math.radians(yaw)
    return np.array([math.cos(pitch) * math.cos(yaw), -math.sin(yaw), math.sin(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.cos(yaw), math.sin(pitch) * math.sin(yaw), -math.sin(pitch), 0, math.cos(pitch)]).reshape(3,3)

def _get_position(px, py, pz, dt):
    global canvas1
    pf, pr, pl, pu = canvas1.view.camera._get_directions()
    rel_speed = dt
    # Create speed vectors, use scale_factor as a reference
    dv = np.array([1.0/d for d in canvas1.view.camera._flip_factors])
    #
    vf = pf * dv * rel_speed * canvas1.view.camera._scale_factor
    vr = pr * dv * rel_speed * canvas1.view.camera._scale_factor
    vu = pu * dv * rel_speed * canvas1.view.camera._scale_factor
    direction = vf, vr, vu

    # Set position
    center_loc = np.array(canvas1.view.camera._center, dtype='float32')
    center_loc += (px * direction[0] +
                   py * direction[1] +
                   pz * direction[2])
    return tuple(center_loc)

if __name__ == "__main__":
    #first thing is to get initial positions from fbgs
    #TCP_Client.startup()
    #TCP_Client.parseData()
    #TCP_Client.tcp()
    #TCP_Client.gradientInit()
    #TCP_Client.closeTCP()
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    vispy.app.run()
    sys.exit(app.exec_())
