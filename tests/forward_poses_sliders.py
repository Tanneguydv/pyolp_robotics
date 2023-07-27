import os
import sys
sys.path.append(os.path.realpath(os.curdir)) 

from OCC.Display.backend import load_backend
load_backend("qt-pyqt5")
import OCC.Display.qtDisplay as qtDisplay

import pyolp_robotics.OCC_functions as occ
from pyolp_robotics.robots.ur import UR_10e

from math import pi, radians, degrees
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(640, 480)
        MainWindow.setStyleSheet("background-color: rgb(77, 77, 77);\n"
        "color: rgb(255, 255, 255);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.sliders_axes_layout = QtWidgets.QVBoxLayout()
        self.sliders_axes_layout.setObjectName("sliders_axes_layout")
        self.gridLayout.addLayout(self.sliders_axes_layout, 0, 0, 1, 1)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.occlayout = QtWidgets.QHBoxLayout()
        self.occlayout.setSizeConstraint(QtWidgets.QLayout.SetMaximumSize)
        self.occlayout.setObjectName("occlayout")
        self.verticalLayout.addLayout(self.occlayout)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout.addItem(spacerItem)
        self.gridLayout.addLayout(self.verticalLayout, 0, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 640, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))

class Application(QtWidgets.QMainWindow):
    def __init__(self, app, parent=None):
        super(Application, self).__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.initialize_gui()
        self.display.FitAll()        

    def initialize_gui(self):
        self.canvas = qtDisplay.qtViewer3d(self)
        self.ui.occlayout.addWidget(self.canvas)
        self.canvas.InitDriver()
        self.display = self.canvas._display
        self.load_robot()

    def move_robot(self, value):
        sender = self.sender() 
        slider_name = list(self.sliders_axes.keys())[list(self.sliders_axes.values()).index(sender)]
        self.robot.config[slider_name] = radians(value)
        self.robot.forward_k(self.robot.config)
        print("config =" , self.robot.config)
        self.show_robot_moving()

    def show_robot_moving(self):
        for i, axe in enumerate(self.robot.get_axes()):
            self.layer_robot.replace_shape(axe, i)
        self.display.View.Redraw()

    def create_sliders_layout(self, nb):
        self.sliders_axes = {}
        self.labels = {}
        for i in range(nb):
            slider = QtWidgets.QSlider()
            slider.setRange(*self.robot.joint_limits[i])
            slider.setOrientation(QtCore.Qt.Orientation(1))  
            slider.setTickPosition(QtWidgets.QSlider.TicksLeft)  
            slider.setTickInterval(5)  
            slider.setSliderPosition(degrees(self.robot.home_config[i]))
            slider.valueChanged.connect(self.update_label_sliders)
            slider.valueChanged.connect(self.move_robot)
            self.sliders_axes[i] = slider

            label = QtWidgets.QLabel()
            label.setText(f"Axe {i}")
            self.labels[i] = label

            self.ui.sliders_axes_layout.addWidget(label)
            self.ui.sliders_axes_layout.addWidget(slider)

    def update_label_sliders(self, value):
        sender = self.sender()  # get the slider that emitted the signal
        slider_name = list(self.sliders_axes.keys())[list(self.sliders_axes.values()).index(sender)]
        self.labels[slider_name].setText(f"Axe {slider_name}: {value}")  # update the label

    def load_robot(self):
        self.robot = UR_10e(mesh=False)
        self.robot.initialise_robot()
        self.robot.go_home()
        self.layer_robot = occ.Layer(self.display, color=self.robot.color, material=self.robot.material)
        self.layer_env = occ.Layer(self.display, self.robot.base_shape, color=self.robot.color, material=self.robot.material)
        self.layer_env.show()
        self.create_sliders_layout(self.robot.nb_axes)
        self.show_robot()

    def show_robot(self):
        for i, axe in enumerate(self.robot.get_axes()):
            self.layer_robot.add_shape(axe)
        self.layer_robot.show()

    def home_robot(self):
        self.robot.set_original_axes()
        for i in range(self.robot.nb_axes):
            self.sliders_axes[i].setValue(0)


if __name__ == "__main__":
    App = QtWidgets.QApplication.instance()

    if App is None:  
        App = QtWidgets.QApplication(sys.argv)
        App.setStyle('Fusion')
        App.lastWindowClosed.connect(App.quit)
    else:
        pass
    MyApp = Application(App)
    MyApp.show()

    try:
        sys.exit(App.exec_())
    except:
        pass