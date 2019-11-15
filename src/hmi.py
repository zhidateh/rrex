#!/usr/bin/env python

import rospy
from rrex.msg import FusedPose as FusedPose
from std_msgs.msg import Float32 

from PyQt5 import QtCore, QtGui, QtWidgets
from mplwidget import MplWidget
import math
import numpy as np
import subprocess
import pathplanning
import sys
from multiprocessing import Process

ldr_value = 0.0

class Ui_MainWindow(object):
        def __init__(self):
                self.interupt = False
                self.pathPlanning = pathplanning.PathPlanning()

                self.state_x = [0.0]
                self.state_y = [0.0]
                                
                self.display_thread = threading.Thread(name = "display_thread",target = self.display, args=[])
                self.move_thread = threading.Thread(name = "move_thread",target = self.move, args=[])
                #self.move_process = Process(target = self.move, name = "move_process", args = ())
                #self.display_process = Process(target = self.display, args = ())
                #self.move_process.daemon = True
                #self.display_process.daemon = True
                

        def setupUi(self, MainWindow):
                MainWindow.setObjectName("MainWindow")
                MainWindow.resize(416, 664)
                self.centralwidget = QtWidgets.QWidget(MainWindow)
                self.centralwidget.setObjectName("centralwidget")
                self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
                self.verticalLayout_2.setObjectName("verticalLayout_2")
                self.MainFrame = QtWidgets.QVBoxLayout()
                self.MainFrame.setObjectName("MainFrame")
                self.init = QtWidgets.QVBoxLayout()
                self.init.setObjectName("init")
                self.btnInit = QtWidgets.QPushButton(self.centralwidget)
                self.btnInit.setObjectName("btnInit")
                self.init.addWidget(self.btnInit)
                self.MainFrame.addLayout(self.init)
                self.radiobox = QtWidgets.QHBoxLayout()
                self.radiobox.setObjectName("radiobox")
                self.rbManual = QtWidgets.QRadioButton(self.centralwidget)
                self.rbManual.setLayoutDirection(QtCore.Qt.LeftToRight)
                self.rbManual.setObjectName("rbManual")
                self.radiobox.addWidget(self.rbManual)
                self.rbLaserTrigger = QtWidgets.QRadioButton(self.centralwidget)
                self.rbLaserTrigger.setObjectName("rbLaserTrigger")
                self.radiobox.addWidget(self.rbLaserTrigger)
                self.MainFrame.addLayout(self.radiobox)
                self.mode = QtWidgets.QHBoxLayout()
                self.mode.setObjectName("mode")
                self.btnManual = QtWidgets.QPushButton(self.centralwidget)
                sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
                sizePolicy.setHorizontalStretch(0)
                sizePolicy.setVerticalStretch(0)
                sizePolicy.setHeightForWidth(self.btnManual.sizePolicy().hasHeightForWidth())
                self.btnManual.setSizePolicy(sizePolicy)
                self.btnManual.setObjectName("btnManual")
                self.mode.addWidget(self.btnManual)
                self.MainFrame.addLayout(self.mode)
                self.estop = QtWidgets.QVBoxLayout()
                self.estop.setObjectName("estop")
                self.btnEStop = QtWidgets.QPushButton(self.centralwidget)
                sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
                sizePolicy.setHorizontalStretch(0)
                sizePolicy.setVerticalStretch(0)
                sizePolicy.setHeightForWidth(self.btnEStop.sizePolicy().hasHeightForWidth())
                self.btnEStop.setSizePolicy(sizePolicy)
                self.btnEStop.setObjectName("btnEStop")
                self.estop.addWidget(self.btnEStop)
                self.MainFrame.addLayout(self.estop)
                self.verticalLayout_2.addLayout(self.MainFrame)
                self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
                self.groupBox.setTitle("")
                self.groupBox.setObjectName("groupBox")
                self.label = QtWidgets.QLabel(self.groupBox)
                self.label.setGeometry(QtCore.QRect(50, 20, 101, 21))
                self.label.setAlignment(QtCore.Qt.AlignCenter)
                self.label.setObjectName("label")
                self.label_2 = QtWidgets.QLabel(self.groupBox)
                self.label_2.setGeometry(QtCore.QRect(220, 20, 161, 21))
                self.label_2.setAlignment(QtCore.Qt.AlignCenter)
                self.label_2.setObjectName("label_2")
                self.txt_spd = QtWidgets.QLabel(self.groupBox)
                self.txt_spd.setGeometry(QtCore.QRect(20, 50, 131, 61))
                font = QtGui.QFont()
                font.setPointSize(48)
                font.setBold(True)
                font.setItalic(True)
                font.setWeight(75)
                self.txt_spd.setFont(font)
                self.txt_spd.setAlignment(QtCore.Qt.AlignCenter)
                self.txt_spd.setObjectName("txt_spd")
                self.txt_dist = QtWidgets.QLabel(self.groupBox)
                self.txt_dist.setGeometry(QtCore.QRect(220, 50, 131, 61))
                font = QtGui.QFont()
                font.setPointSize(48)
                font.setBold(True)
                font.setItalic(True)
                font.setWeight(75)
                self.txt_dist.setFont(font)
                self.txt_dist.setAlignment(QtCore.Qt.AlignCenter)
                self.txt_dist.setObjectName("txt_dist")
                self.line = QtWidgets.QFrame(self.groupBox)
                self.line.setGeometry(QtCore.QRect(190, 30, 20, 81))
                self.line.setFrameShape(QtWidgets.QFrame.VLine)
                self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
                self.line.setObjectName("line")
                self.verticalLayout_2.addWidget(self.groupBox)
                self.verticalLayout = QtWidgets.QVBoxLayout()
                self.verticalLayout.setObjectName("verticalLayout")
                
                self.plotwidget = MplWidget(self.centralwidget)
                #self.plotwidget = QtWidgets.QWidget(self.centralwidget)
                self.plotwidget.setAutoFillBackground(False)
                self.plotwidget.setObjectName("plotwidget")
                self.verticalLayout.addWidget(self.plotwidget)
                self.verticalLayout_2.addLayout(self.verticalLayout)
                self.verticalLayout_2.setStretch(0, 3)
                self.verticalLayout_2.setStretch(1, 2)
                self.verticalLayout_2.setStretch(2, 5)
                MainWindow.setCentralWidget(self.centralwidget)
                self.menubar = QtWidgets.QMenuBar(MainWindow)
                self.menubar.setGeometry(QtCore.QRect(0, 0, 416, 27))
                self.menubar.setObjectName("menubar")
                MainWindow.setMenuBar(self.menubar)
                self.statusbar = QtWidgets.QStatusBar(MainWindow)
                self.statusbar.setObjectName("statusbar")
                MainWindow.setStatusBar(self.statusbar)

                self.retranslateUi(MainWindow)
                QtCore.QMetaObject.connectSlotsByName(MainWindow)
                self.btnManual.setEnabled(False)
                self.btnEStop.setEnabled(False)
                
                self.setupSignalsnSlots()
                app.aboutToQuit.connect(self.closeEvent)
                
                self.display_thread.start()
                self.rbManual.setChecked(True)
                
                #self.display_process.start()

        def setupSignalsnSlots(self):
                self.btnInit.clicked.connect(self.btnInit_clicked)
                self.btnEStop.clicked.connect(self.btnEStop_clicked)
                self.btnManual.clicked.connect(self.btnManual_clicked)


        def retranslateUi(self, MainWindow):
                _translate = QtCore.QCoreApplication.translate
                MainWindow.setWindowTitle(_translate("MainWindow", "RREX User Interface"))
                self.btnInit.setText(_translate("MainWindow", "PRESS ME"))
                self.rbManual.setText(_translate("MainWindow", "Manual"))
                self.rbLaserTrigger.setText(_translate("MainWindow", "Laser Trigger"))
                self.btnManual.setText(_translate("MainWindow", "Start"))
                self.btnEStop.setText(_translate("MainWindow", "E-STOP"))
                self.label.setText(_translate("MainWindow", "Speed [m/s] :"))
                self.label_2.setText(_translate("MainWindow", "Distance Travelled [m] :"))
                self.txt_spd.setText(_translate("MainWindow", "0"))
                self.txt_dist.setText(_translate("MainWindow", "0"))

        def btnInit_clicked(self):
                
                try:
                        options = QtWidgets.QFileDialog.Options()
                        options |= QtWidgets.QFileDialog.DontUseNativeDialog
                        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self.centralwidget,"Select path","/share/","All Files (*);;Text Files (*.txt)", options=options)
                        
                        if(fileName[-4:] == ".txt"):
                                with open(fileName, 'r') as f:
                                        cx,cy = [],[]
                                        for row in f:
                                            cx.append(float(row.split(',')[0]))
                                            cy.append(float(row.split(',')[1]))
                        else:
                                print "\tPath selection - failed"
                                return
                        
                        robot_y = [-0.24,0.24,0.24,-0.24,-0.24]
                        robot_x = [0, 0 ,-0.48,-0.48,0] 
                        self.plotwidget.canvas.ax.clear()
                        self.plotwidget.canvas.ax.plot(cy,cx)
                        self.plotwidget.canvas.ax.plot(robot_y,robot_x)
                        self.plotwidget.canvas.ax.legend((' robot' , 'path'),loc='upper right')
                        self.plotwidget.canvas.ax.set_title('Trajectory')
                        #x axis is y plot, y axis is x plot, so that x is in longitudinal direction
                        self.plotwidget.canvas.ax.set_xlim(xmin=-5, xmax=5)
                        self.plotwidget.canvas.ax.set_ylim(ymin= -1, ymax=9)
                        self.plotwidget.canvas.draw()
                        
                        print "\n***Copying path to ~/catkin_ws/src/rrex/config/path.txt ..."
                        msg = "sudo cp " + fileName + " ~/catkin_ws/src/rrex/config/path.txt"
                        subprocess.call([msg],shell=True)
                        print "\tPath selection - success"
                        
                        self.btnManual.setEnabled(True)
                        self.btnEStop.setEnabled(True)


                except:
                        print "\tPath selection - failed"
                        pass
                        
        def btnEStop_clicked(self):

                print "\n***Killing system immediately...\n"
                self.pathPlanning.interupt = True
                
                subprocess.call(['sudo pkill roslaunch'],shell=True)
                subprocess.call(['sudo killall pigpiod'],shell=True)    
                subprocess.call(['sudo pkill python'],shell=True)

                self.btnManual.setEnabled(False)
                self.btnEStop.setEnabled(False)
                self.interupt = True
                
                self.pathPlanning.plot()

                QtWidgets.QApplication.quit()

        def closeEvent(self):
                print "\n***Killing system immediately...\n"
                self.pathPlanning.interupt = True
                subprocess.call(['sudo pkill roslaunch'],shell=True)
                subprocess.call(['sudo pkill python'],shell=True)
                self.interupt = True
                QtWidgets.QApplication.quit()

                
        def btnManual_clicked(self):
                #self.pathPlanning.trigger()
                if(self.rbManual.isChecked):
                        self.move_thread.start()
                elif (self.rbLaserTrigger.isChecked):
                        if(ldr_value < 3.5):
                                self.move_thread.start()
                        else:
                                btnManual_clicked()
                                
                                

        def move(self):
                self.pathPlanning.trigger()
        
        
        def display(self):
                while not self.interupt:
                        if (ldr_value < 3.5):
                                self.rbLaserTrigger.setText("Laser Trigger [1]")
                        else:
                                self.rbLaserTrigger.setText("Laser Trigger [0]")
                
        
                        self.state_x.append(pathplanning.state_x)
                        self.state_y.append(pathplanning.state_y)
                        state_dis = 0.0
                        for i in range(0,len(self.state_x)-1):
                                dx = self.state_x[i+1] - self.state_x[i] 
                                dy = self.state_y[i+1] - self.state_y[i]
                                state_dis += math.hypot(dx,dy)

                        self.txt_spd.setText(str('{0:.1f}'.format(pathplanning.state_vel)))
                        self.txt_dist.setText(str('{0:.1f}'.format(state_dis)))
 
                        if(math.isnan(state_dis) or math.isnan(pathplanning.state_vel) or np.isinf(state_dis) or np.isinf(pathplanning.state_vel)):
                                self.closeEvent()
        
def ldr_callback(data):
        global ldr_value
        ldr_value = data.data
        

def init_rrex():
        print "\n***Initialising RREX robot architecture..."
        subprocess.call(['bash /home/pi/rrex-setup.bash'],shell=True)


if __name__ == "__main__":

        import sys
        import threading
        
        subprocess.call(['sudo pigpiod'],shell=True)
        rrex_thread = threading.Thread(name = "rrex_thread",target = init_rrex,args=[])
        rrex_thread.start()
        #rrex_process = Process(name = "rrex_process",target=init_rrex, args =[])
        #rrex_process.daemon=True
        #rrex_process.start()
        
        rospy.Subscriber("/ldr_node/ldr", Float32, ldr_callback)

        app = QtWidgets.QApplication(sys.argv)
        MainWindow = QtWidgets.QMainWindow()
        ui = Ui_MainWindow()
        ui.setupUi(MainWindow)
        MainWindow.show()
        sys.exit(app.exec_())

