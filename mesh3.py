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

LINE = None
ui = None
startTime = 0
testNum = 0
xyz = None
view = None
initalPose = None
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
        self.view.camera.fov = 90
        self.view.camera.scale_factor = 2.0
        self.view.camera.interactive = True
        self.view.camera.zoom_factor = 0
        self.view.camera.auto_roll = False
        #view.camera.distance = 0
        #view.camera.center = (-.491, -.493, .1508)
        #os.abort()
        #Keys: L is pan right; j is pan right; i is pan up; k is pan down
        #view.camera.center = (-73.979935, -63.755707, 10.842972)
        varMesh = trimesh.load(file)
        print(str(varMesh.vertices))
        #maxValue = np.max(np.array(mesh.vertices))
        #print(str(maxValue))
        #mesh.apply_scale(1/maxValue)
        print(str(varMesh.vertices))
        #os.abort()
        #group = EmitterGroup(source=None, auto_connect=False)
        #mouseEvent = EventEmitter(Event("mouse_press"), view.scene)
        #print(str(type(mouseEvent)))
        #group.add(mouse_press=mouseEvent)
        #group.connect(self.needleStart)
        self.connect(self.on_key_press)
        mdata = geometry.MeshData(varMesh.vertices, varMesh.faces)
        print(str(varMesh.vertices))
        self.meshes.append(Mesh(meshdata=mdata, shading='smooth', color=(1,0,0,.6), parent=self.view.scene))
        self.timer = vispy.app.Timer(connect = self.line)
        self.timer.start(0.001)
        #self.timer = app.Timer(connect=self.pos)
        #self.timer.start(.001)
        #ax = AxisWidget()
        finalArr = list(self.view.camera.center)
        initalPose = list(self.view.camera.center)
        for i in range(0, len(finalArr)):
             finalArr[i] = finalArr[i] + 1
        x1 = initalPose[0]
        x2 = finalArr[0]
        y1 = initalPose[1]
        y2 = finalArr[1]
        z1 = initalPose[2]
        z2 = finalArr[2]
        xyz = XYZAxis(pos = np.array([[x1, y1, z1], [x2, y1, z1], [x1, y1, z1], [x1, y2, z1], [x1, y1, z1], [x1, y1, z2]]), parent=self.view.scene)

        #self.freeze()
        arr = [[-57, -47, 21], [-57, -47, 21], [-53, -47, 21], [-50, -47, 21], [-47, -47, 21], [-44, -47, 21], [-41, -47, 21], [-38, -47, 21], [-34, -47, 21], [-31, -47, 21]]
        LINE = Plot3D(arr, width=2, color='red', edge_color='w', symbol='o', face_color=(.2, .2, 1, .8), parent=self.view.scene)

    def on_key_press(self, key):
        global initialPoseBool
        global view
        global initalPose
        if ord(key.text) == 13:
            print(str(self.view.camera.center))

        if ord(key.text) == 13 and initialPoseBool == False:
            initalPose = self.view.camera.center
            initialPoseBool = True
        if ord(key.text) == 43:
            self.view.camera.scale_factor = self.view.camera.scale_factor + 1
        if ord(key.text) == 45:
            if self.view.camera.scale_factor - 1 > 0:
                self.view.camera.scale_factor = self.view.camera.scale_factor - 1

    def needleStart(self):
        print("HERE")

    def pos(self, event):
        global view
        print(str(self.view.camera.center))

    def line(self, event):
        global startTime
        global view
        global xyz
        global testNum
        global LINE
        global Plot3D
        testNum += 1
        #LINE.parent = None
        arr = [[-57, -47, 21], [-57, -47, 21], [-53, -47, 21], [-50, -47, 21], [-47, -47, 21], [-44, -47, 21], [-41, -47, 21], [-38 + testNum, -47, 21], [-34 + testNum, -47, 21], [-31 + testNum, -47, 21]]
        finalArr = list(self.view.camera.center)
        initalPose = list(self.view.camera.center)
        for i in range(0, len(finalArr)):
            initalPose[i] = initalPose[i] + .1
            finalArr[i] = finalArr[i] + .3
        x1 = initalPose[0]
        x2 = finalArr[0]
        y1 = initalPose[1]
        y2 = finalArr[1]
        z1 = initalPose[2]
        z2 = finalArr[2]
        if testNum % 1000 == 0:
            print("HERE")
            xyz.parent = None
        #xyz = XYZAxis(pos = np.array([[x1, y1, z1], [x2, y1, z1], [x1, y1, z1], [x1, y2, z1], [x1, y1, z1], [x1, y1, z2]]), parent=view.scene)
            xyz = XYZAxis(pos = np.array([[x1, y1, z1], [x2, y1, z1], [x1, y1, z1], [x1, y2, z1], [x1, y1, z1], [x1, y1, z2]]), parent=self.view.scene)
            self.view.update()
        #LINE = Plot3D(arr, width=2, color='red', edge_color='w', symbol='o', face_color=(.2, .2, 1, .8), parent=view.scene)
        #view.camera.rotation1 = q
        #if testNum <= .01:
        #    view.camera.view_changed()
        #    print("HERE")
        #print("\n\n")
        #print(str(view.camera.rotation1))
        #print(str(view.camera.rotation2))
        #print(str(view.camera.center))
        #print("\n")
        #print(str(view.camera.transform))
        #print(str(view.camera.get_range()))
        #iew.camera.rotation2 = q
        #up, forward, right = view.camera._get_dim_vectors()
        #print(dir(view.camera.transform))
        #print(dir(view.camera))
        #view.camera._update_projection_transform(10,10)
        #os.abort()
        #print(view.camera.transform.rotate(0, axis=(1,0,0)))
        #print(view.camera.transform.rotate(.1, axis=(0,1,0)))
        #print(str(view.camera.rotation1))
        #print(str(view.camera.rotation2))
        #view.camera.rotation1 = q
        #view.camera.rotation2 = q
        #self.update()

    #view.camera.center = (view.camera.center[0] + .1, view.camera.center[1] + .1, view.camera.center[2] - .1)
    #looks like it is possible to set rotation (must be quaterion)
    #print(str(view.camera.rotation1))
    #print(str(view.camera.rotation2))

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
        self.pushButton.hide()
        self.pushButton1.hide()
        self.label.hide()
        canvas1 = MyCanvas()
        file = r"C:\Users\Birth\OneDrive\Documents\BIRTH\heart1.stl"
        canvas2 = MyCanvas()
        lay = QGridLayout() #Choose gridlayout for vispy
        self.centralwidget.setLayout(lay)
        #lay.addWidget(Color('red'), 0, 0, 1, 3) # y, x, span along y, span along x
        lay.addWidget(Color('red'), 0, 0, 1, 4)
        lay.addWidget(canvas1, 1, 0, 3, 2)
        lay.addWidget(canvas2, 1, 2, 3, 2)
        self.pushButton.hide()


def end():
    print("STOPPING EVERYTHING")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    vispy.app.run()
    sys.exit(app.exec_())
