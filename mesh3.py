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
import TCP_Client 
import trimesh
from vispy.util.event import *
import vispy.scene
from vispy.scene.visuals import XYZAxis
from vispy.scene.widgets.axis import AxisWidget
import time
import os
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
import pythonping
from functools import partial
import pathlib
import subprocess
from vispy.app.qt import QtSceneCanvas
from scipy.spatial.transform import Rotation as R

LINE = None
ui = None
startTime = 0
testNum = 0
xyz = None
view = None
initialPose = None
initialNeedleRotation = None 
initialRotation = None 
initialPoseBool = False
Plot3D = None
file = ""

class MyCanvas(vispy.app.qt.QtSceneCanvas):

    def __init__(self):
        global file
        global view
        global LINE
        global Plot3D
        global xyz
        vispy.app.qt.QtSceneCanvas.__init__(self, keys='interactive', size=(1080, 720), bgcolor=(0,0,0,0))
        self.unfreeze()
        Plot3D = scene.visuals.create_visual_node(visuals.LinePlotVisual)
        self.meshes = []
        self.view = self.central_widget.add_view()
        self.view.camera = 'fly'
        self.firstView = True 
        self.view.camera.fov = 90
        self.view.camera.scale_factor = 2.0
        self.view.camera.interactive = True
        self.view.camera.zoom_factor = 0
        self.view.camera.auto_roll = False
        varMesh = trimesh.load(file)
        print(str(varMesh.vertices))
        print(str(varMesh.vertices))
        self.connect(self.on_key_press)
        mdata = geometry.MeshData(varMesh.vertices, varMesh.faces)
        print(str(varMesh.vertices))
        self.meshes.append(Mesh(meshdata=mdata, shading='smooth', color=(1,0,0,.6), parent=self.view.scene))
        self.timer = vispy.app.Timer(connect = self.line)
        self.timer.start(0.01)

    def on_key_press(self, key):
        global initialPoseBool
        global view
        global initialPose
        global initialNeedleRotation
        global initialRotation
        if ord(key.text) == 13:
            print(str(self.view.camera.center))
        if ord(key.text) == 13 and initialPoseBool == False:
            initialPose = self.view.camera.center
            initialRotation = self.view.rotation1 
            initialNeedleRotation = TCP_Client.tcp()
            initialPoseBool = True
        if ord(key.text) == 43:
            self.view.camera.scale_factor = self.view.camera.scale_factor + 1
        if ord(key.text) == 45:
            if self.view.camera.scale_factor - 1 > 0:
                self.view.camera.scale_factor = self.view.camera.scale_factor - 1

    def line(self, event):
        global startTime
        global view
        global xyz
        global testNum
        global LINE
        global initialPose
        global Plot3D
        global initialRotation

        if self.firstView == True: 
            rpy_ = TCP_Client.tcp()
            xPoints, yPoints, zPoints = TCP_Client.graphNeedle() #Last element in returned list is the 3d point representation of the needle tip
            length_ = len(xPoints) - 1
            self.view.camera.center = (xPoints[length_] + initialPose[0], yPoints[length_] + initialPose[1], zPoints[length_] + initialPose[2])
            changeP = rpy_[1] - initialNeedleRotation[1]
            changeY = rpy_[2] - initialNeedleRotation[2]
            #rpy2quat returns a quaternion in x,y,z,w format 
            #Vispy Quaternion constructor is in w,x,y,z format
            #Might need to call Quaternion.conjugate if the rpy2quat is opposite 
            RPY_Quat = rpy2quat([0, changeP, changeY], False)
            needleQuat = Quaternion(w=RPY_Quat[3], x=RPY_Quat[0], y=RPY_Quat[1], z=RPY_Quat[2]) 
            newQuatArr = needleQuat * initialRotation #Quat multiplication necessary to add two together 
            self.view.camera.rotation1 = newQuatArr
            self.view.camera.view_changed()
            
        else: 
            arr = []
            LINE.parent = None 
            xPoints, yPoints, zPoints = TCP_Client.graphNeedle()
            for i in range(0, len(xPoints)):
                temp = [xPoints[i] + initialPose[0], yPoints[i] + initialPose[1], zPoints[i] + initialPose[2]]
                arr.append(temp) 
            LINE = Plot3D(arr, width=2, color='blue', edge_color='white', symbol='o', face_color=(.2, .2, 1, .8), parent=self.view.scene)

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
        self.pushButton.hide()
        self.pushButton1.hide()
        self.label.hide()
        canvas1 = MyCanvas()
        canvas2 = MyCanvas()
        TCP_Client.startup()    #Startup TCP. This may not be the right line to put this 
        TCP_Client.parseData()
        lay = QGridLayout() #Choose gridlayout for vispy
        self.centralwidget.setLayout(lay)
        #lay.addWidget(Color('red'), 0, 0, 1, 3) # y, x, span along y, span along x
        lay.addWidget(Color('red'), 0, 0, 1, 4)
        lay.addWidget(canvas1, 1, 0, 3, 2)
        lay.addWidget(canvas2, 1, 2, 3, 2)
        self.pushButton.hide()

def rpy2quat(rpy, degrees=False):
    r = R.from_euler('ZYX', rpy, degrees=degrees)
    return r.as_quat()

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    vispy.app.run()
    sys.exit(app.exec_())
