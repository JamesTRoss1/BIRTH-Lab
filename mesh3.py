import numpy as np
import vispy
from vispy import app, gloo, geometry, io, visuals, scene
vispy.use('PyQt5')
from vispy.geometry import create_sphere
import math
from vispy.scene.visuals import Mesh
from vispy.visuals.transforms import (STTransform, MatrixTransform,
                                  ChainTransform)
from vispy.util.quaternion import Quaternion
from vispy.scene.events import SceneMouseEvent
import trimesh
import TCP_Client
from vispy.util.event import *
import vispy.scene
from vispy.scene.visuals import XYZAxis
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

class MyCanvas(vispy.app.qt.QtSceneCanvas):

    def __init__(self, firstView = True):
        global file
        global view
        global LINE
        global Plot3D
        global xyz
        global wireframe_filter
        global shading_filter
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
        self.view.camera.scale_factor = 2.0
        self.view.camera.zoom_factor = 0
        self.view.camera.center = (0,0,0)
        #x:90-forward is z; y:90- forward x; z:90-forward y
        self.view.camera.rotation1 = Quaternion.create_from_euler_angles(90,90,0, True)
        if not firstView:
            self.view.camera.interactive = True
            self.view.camera.auto_roll = True
        else:
            self.view.camera.interactive = False
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
            self.mesh = Mesh(meshdata=mdata, shading='flat', color=(1,0,0,.6), parent=self.view.scene)
        if not firstView:
            self.mesh = Mesh(meshdata=mdata, shading=None, color=(1,1,1,1), parent=self.view.scene)
            wireframe_filter = WireframeFilter(wireframe_only=True, color='white', width=2)
            self.mesh.attach(wireframe_filter)
            #self.mesh.attach(ColorFilter(filter = (.2, .2, 1, 1)))
            #self.mesh.attach(Alpha(.7))
            shading_filter = ShadingFilter(shading='flat', diffuse_light=(1,1,1,.9))
            self.mesh.attach(shading_filter)
        self.timer = vispy.app.Timer(connect = self.wait)
        self.timer.start(0.1)
        finalArr = list(self.view.camera.center)
        initialPose = list(self.view.camera.center)

    def wait(self, event):
        print("Waiting")

    def on_key_press(self, key):
        global initialPoseBool
        global view
        global initialRotation
        global initialNeedleRotation
        global initialPose
        global canvas1
        global canvas2
        global wireframe_filter
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
            initialNeedleRotation = (0, 0, 0) #Call tcp angles before running
            TCP_Client.startup()
            TCP
            canvas1.timer.stop()
            canvas1.timer = vispy.app.Timer(connect = canvas1.line)
            canvas1.timer.start(0.05)
            self.timer.stop()
            print("ID FROM THE ENTER: " + str(id(self)))

        if ord(key.text) == 43:
            self.view.camera.scale_factor = self.view.camera.scale_factor + 1
        if ord(key.text) == 45:
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
        counter = counter + 5
        if counter % 100 != 0:
            return
        xPoints = []
        yPoints = []
        zPoints = []
        #TCP_Client_Test.testTcp(counter // 100)
        #points, origin = TCP_Client_Test.needleBySlope() #first 3 elements in returned list is the 3d point representation of the needle tip; last is new origin
        TCP_Client.tcp()
        points = TCP_Client.needleByGradient()

        #MAKE SURE THAT THE POINTS MATCH THE COORDINATE FRAME OF THE VISPY CANVAS
        for point in range(0, len(points)):
            xPoints.append(points[point][0])
            yPoints.append(points[point][1])
            zPoints.append(points[point][2])
        length_ = len(xPoints) - 1
        self.view.camera.center = (xPoints[length_] + initialPose[0], yPoints[length_] + initialPose[1], zPoints[length_] + initialPose[2])
        #rpy_ = TCP_Client_Test.angles()
        rpy_ = (0, counter // 100, counter // 100)
        changeP = rpy_[1] - initialNeedleRotation[1]
        changeY = rpy_[2] - initialNeedleRotation[2]
            #rpy2quat returns a quaternion in x,y,z,w format
            #Vispy Quaternion constructor is in w,x,y,z format
            #Might need to call Quaternion.conjugate if the rpy2quat is opposite
        #rx has object appear to move in a counterclockwise direction
        #ry is side to side; positive toward right
        #rz is up and down; positive toward down
        #x forward is positive
        #y to the left is positive
        #z upwards is positive
        needleQuat = Quaternion.create_from_euler_angles(0, changeP, changeY, True)
        newQuatArr = needleQuat * initialRotation #Quat multiplication necessary to add two together
        self.view.camera.rotation1 = newQuatArr
        self.view.camera.view_changed()

        #CANVAS 2 LOGIC
        arr = []
        if LINE is not None:
            LINE = None
        for i in range(0, len(xPoints)):
            temp = [xPoints[i] + initialPose[0], yPoints[i] + initialPose[1], zPoints[i] + initialPose[2]]
            arr.append(temp)
        LINE = Plot3D(arr, width=3, color=(0,0,1,1), edge_color=(1, 1, 1, 1), symbol='o', face_color=(.2, .2, 1, 1), parent=canvas2.view.scene)
        LINE.order = 2
        canvas2.mesh.mesh_data_changed()
        canvas2.view.camera.view_changed()

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
        print("CANVAS 1: " + str(id(canvas1)))
        print("CANVAS 2: " + str(id(canvas2)))
        file = r"C:\Users\Birth\OneDrive\Documents\BIRTH\heart1.stl"
        lay = QGridLayout() #Choose gridlayout for vispy
        self.centralwidget.setLayout(lay)
        #lay.addWidget(Color('red'), 0, 0, 1, 3) # y, x, span along y, span along x
        lay.addWidget(Color('red'), 0, 0, 1, 4)
        lay.addWidget(canvas2, 1, 2, 3, 2)
        lay.addWidget(canvas1, 1, 0, 3, 2)
        self.pushButton.hide()


def end():
    print("STOPPING EVERYTHING")

if __name__ == "__main__":
    #first thing is to get initial positions from fbgs
    TCP_Client.startup()
    TCP_Client.parseData()
    TCP_Client.tcp()
    TCP_Client.gradientInit()
    TCP_Client.closeTCP()
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    vispy.app.run()
    sys.exit(app.exec_())
