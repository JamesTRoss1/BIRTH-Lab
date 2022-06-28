from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import sys
import os
import pythonping
from functools import partial
import pathlib
import subprocess

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

        #Future Idea: implememnt a combo box with button in order to provide user access to parameters for conversion file (dicom2stl.py)

        if dcm_files.suffix == ".zip":
            name = dcm_files.stem
            new_file = name + ".stl"
            command = "python dicom2stl.py -t tissue -o " + new_file + " " + dcm_files
            ret = subprocess.call(command, shell=True)  #convert the zip of dicom files and then save as same file with extension .stl
            #find file and create File IO wrapper for file


        png, done4 = QFileDialog.getOpenFileName(None, "Choose JPG or PNG file for top-down view", "", "(*.png);;(*.jpg)")
        png = str(png)
        self.label.setText("Robot IP Address: " + str(ip) + "\n" +
                            "Needle Offset: " + str(offset) + "\n" +
                            "Dicom or STL File: " + str(dicom_file) + "\n" +
                            "JPG or PNG File: " + str(png))
        self.label.adjustSize()
        self.pushButton1 = QPushButton(self.centralwidget)
        self.pushButton1.setText("Restart")
        self.pushButton.setText("Continue")
        self.pushButton.setGeometry(150, 250, 100, 100)
        self.pushButton1.setGeometry(400, 250, 100, 100)
        take_input = partial(self.takeinputs, MainWindow)
        self.pushButton1.clicked.connect(take_input)
        self.pushButton.clicked.disconnect()
        finish = partial(self.finished_input, ip = str(ip), offset = str(offset), file = str(dicom_file))
        self.pushButton.clicked.connect(finish)
        self.pushButton1.show()

    def finished_input(self, canvas, file, ip = "", offset = 0):
        self.pushButton.hide()
        self.pushButton1.hide()
        self.label.hide()
        lay = QGridLayout() #Choose gridlayout for vispy
        self.centralwidget.setLayout(lay)
        #lay.addWidget(Color('red'), 0, 0, 1, 3) # y, x, span along y, span along x
        lay.addWidget(Color('red'), 0, 0, 1, 4)
        lay.addWidget(canvas.native, 1, 0, 3, 2)
        lay.addWidget(Color('black'), 1, 2, 3, 2)
        self.pushButton.hide()


def end():
    print("STOPPING EVERYTHING")


if __name__ == "__main__":
	app = QApplication(sys.argv)
	MainWindow = QMainWindow()
	ui = Ui_MainWindow()
	ui.setupUi(MainWindow)
	MainWindow.show()

	sys.exit(app.exec_())
