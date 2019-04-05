import sys
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5 import QtWidgets,QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

#from mymodules import fk_module
#from mymodules import Closed_form_ik
from Arm import Arm

from utils import *

import glob
import socket
import math


class Application(QtWidgets.QWidget):

    def __init__(self,arm):

        # super init
        super(Application, self).__init__()

        # link to the arm
        self.arm = arm

        #init UI
        self.init_UI()

        # init figure
        self.init_figure()

        # init menu
        self.init_menu()

    def init_UI(self):
        """
        initialisation of the UI windows
        """
        self._count = True
        # Create Widget for Figure/Menu
        self.FigureWidget = QtWidgets.QWidget(self)
        self.MenuWidget = QtWidgets.QWidget(self)

        # Add Layout to each Widget
        self.FigureLayout = QtWidgets.QVBoxLayout(self.FigureWidget)
        self.MenuLayout = QtWidgets.QVBoxLayout(self.MenuWidget)

        # Delete Margin of the layout
        self.FigureLayout.setContentsMargins(0, 0, 0, 0)
        self.MenuLayout.setContentsMargins(0, 0, 0, 0)

        # set Geometry
        self.setGeometry(0, 0, 900, 600)
        self.FigureWidget.setGeometry(400, 0, 700, 600)
        self.MenuWidget.setGeometry(0, 0, 200, 600)

        return 0

    def init_figure(self):
        """
        initialisation of the figure
        """
        ## Creat the Figure to drow 3D plot
        self.Figure = plt.figure()
        # Add FigureCanvas to Figure
        self.FigureCanvas = FigureCanvas(self.Figure)
        # Addd FigureCanvas to Layout
        self.FigureLayout.addWidget(self.FigureCanvas)
        self.axis = self.Figure.add_subplot(121, projection='3d')

        self.axis_xlabel = "X-axis"
        self.axis_ylabel = "Y-axis"
        self.axis_zlabel = "Z-axis"
        self.axis_xlim3d = [-0.2, 0.2]
        self.axis_ylim3d = [-0.2, 0.2]
        self.axis_zlim3d = [-0.2, 0.2]

        # set the label for each axis
        self.axis.set_xlabel(self.axis_xlabel)
        self.axis.set_ylabel(self.axis_ylabel)
        self.axis.set_zlabel(self.axis_zlabel)
        # set the limit of each axis
        self.axis.set_xlim3d(self.axis_xlim3d[0], self.axis_xlim3d[1])
        self.axis.set_ylim3d(self.axis_ylim3d[0], self.axis_ylim3d[1])
        self.axis.set_zlim3d(self.axis_zlim3d[0], self.axis_zlim3d[1])

        return 0

    def init_menu(self):
        """
        Initialisation of the menu
        """
        # Create the label objects

        self.inputlabel = QtWidgets.QLabel(self)
        self.outputlabel = QtWidgets.QLabel(self)

        self.xlabel = QtWidgets.QLabel(self)
        self.ylabel = QtWidgets.QLabel(self)
        self.zlabel = QtWidgets.QLabel(self)

        self.th0label = QtWidgets.QLabel(self)
        self.th1label = QtWidgets.QLabel(self)
        self.th2label = QtWidgets.QLabel(self)
        self.th3label = QtWidgets.QLabel(self)
        self.th4label = QtWidgets.QLabel(self)
        self.th5label = QtWidgets.QLabel(self)

        self.deg0label = QtWidgets.QLabel(self)
        self.deg1label = QtWidgets.QLabel(self)
        self.deg2label = QtWidgets.QLabel(self)
        self.deg3label = QtWidgets.QLabel(self)
        self.deg4label = QtWidgets.QLabel(self)
        self.deg5label = QtWidgets.QLabel(self)

        # Create the textbox objects
        self.xtextbox = QtWidgets.QLineEdit(self)
        self.ytextbox = QtWidgets.QLineEdit(self)
        self.ztextbox = QtWidgets.QLineEdit(self)

        self.deg0textbox = QtWidgets.QLineEdit(self)
        self.deg1textbox = QtWidgets.QLineEdit(self)
        self.deg2textbox = QtWidgets.QLineEdit(self)
        self.deg3textbox = QtWidgets.QLineEdit(self)
        self.deg4textbox = QtWidgets.QLineEdit(self)
        self.deg5textbox = QtWidgets.QLineEdit(self)

        # Create the button objects
        self.drowbutton = QtWidgets.QPushButton('draw', self)

        # Resize text box
        self.xtextbox.resize(100, 20)
        self.ytextbox.resize(100, 20)
        self.ztextbox.resize(100, 20)

        self.deg0textbox.resize(100, 20)
        self.deg1textbox.resize(100, 20)
        self.deg2textbox.resize(100, 20)
        self.deg3textbox.resize(100, 20)
        self.deg4textbox.resize(100, 20)
        self.deg5textbox.resize(100, 20)

        # set the size of the button object
        self.drowbutton.resize(100, 30)

        # set the name of each label
        self.inputlabel.setText('INPUT')
        self.outputlabel.setText('OUTPUT')

        self.xlabel.setText('x')
        self.ylabel.setText('y')
        self.zlabel.setText('z')

        self.th0label.setText('th0')
        self.th1label.setText('th1')
        self.th2label.setText('th2')
        self.th3label.setText('th3')
        self.th4label.setText('th4')
        self.th5label.setText('th5')

        self.deg0label.setText('[deg]')
        self.deg1label.setText('[deg]')
        self.deg2label.setText('[deg]')
        self.deg3label.setText('[deg]')
        self.deg4label.setText('[deg]')
        self.deg5label.setText('[deg]')

        # set the location of each labels
        pos = [15, 30, 50, 30, 200, 30, 255, 30]

        self.inputlabel.move(15, 10)

        self.th0label.move(15, 30)
        self.th1label.move(15, 30 + 30)
        self.th2label.move(15, 30 + 60)
        self.th3label.move(15, 30 + 90)
        self.th4label.move(15, 30 + 120)
        self.th5label.move(15, 30 + 150)

        self.deg0textbox.move(50, 30)
        self.deg1textbox.move(50, 30 + 30)
        self.deg2textbox.move(50, 30 + 60)
        self.deg3textbox.move(50, 30 + 90)
        self.deg4textbox.move(50, 30 + 120)
        self.deg5textbox.move(50, 30 + 150)

        self.deg0label.move(150, 30)
        self.deg1label.move(150, 30 + 30)
        self.deg2label.move(150, 30 + 60)
        self.deg3label.move(150, 30 + 90)
        self.deg4label.move(150, 30 + 120)
        self.deg5label.move(150, 30 + 150)

        self.outputlabel.move(15, 230)

        self.xlabel.move(  15, 250)
        self.ylabel.move(  15, 250 + 30)
        self.zlabel.move(  15, 250 + 60)
        self.xtextbox.move(50, 250)
        self.ytextbox.move(50, 250 + 30)
        self.ztextbox.move(50, 250 + 60)

        self.drowbutton.move(50, 200 + 180)

        #connect the button to the click_action
        self.drowbutton.clicked.connect(self.on_drowclick)


        return 0

    def on_drowclick(self):
        """
        method linked to the button draw
        """
        q = deg2rad(np.array(self.get_input()).astype(float))
        print(q)
        if not self.check_input(q):
            # no valid input
            return 1
        else:
            # valid input
            R,t = self.arm.kinematic.compute_FK(q) # compute the total kinematics
            x,y,z = self.arm.kinematic.compute_pose(q) # compute all partial kinematics
            self.set_output([t[0], t[1], t[2]]) # set the output
            list_ref = self.arm.kinematic.compute_ref(q)  # compute all the local ref

            self.clear_figure() # clear figure
            self.draw_arm([x, y, z]) # draw the arm
            #self.draw_ref(list_ref)
            self.FigureCanvas.draw()



        return 0

    def get_input(self):
        """
        getter of the input values
        :return: return the input values
        """
        th0 = self.deg0textbox.text()
        th1 = self.deg1textbox.text()
        th2 = self.deg2textbox.text()
        th3 = self.deg3textbox.text()
        th4 = self.deg4textbox.text()
        th5 = self.deg5textbox.text()

        if th0 == '':
            th0 = '0'
            self.deg0textbox.setText('0')

        if th1 == '':
            th1 = '0'
            self.deg1textbox.setText('0')

        if th2 == '':
            th2 = '0'
            self.deg2textbox.setText('0')

        if th3 == '':
            th3 = '0'
            self.deg3textbox.setText('0')

        if th4 == '':
            th4 = '0'
            self.deg4textbox.setText('0')

        if th5 == '':
            th5 = '0'
            self.deg5textbox.setText('0')


        return th0, th1, th2, th3, th4, th5

    def check_input(self, q):
        """
        this method check if the input are allowed
        :return: true or false depend of the input
        """
        ANGLE_LIMIT_INF = self.arm.ANGLE_LIMIT_INF
        ANGLE_LIMIT_SUP = self.arm.ANGLE_LIMIT_SUP

        for i in range(len(q)):
            if q[i] < ANGLE_LIMIT_INF[i] or q[i] > ANGLE_LIMIT_SUP[i]:
                print('theta {} is not is the acceptable range {} given, range : [{} {}]'.format(i,q[i],ANGLE_LIMIT_INF[i],ANGLE_LIMIT_SUP[i]))
                return False

        return True

    def set_output(self, X):
        """
        method to set the output text box
        :param X: list [x,y,z]
        """

        self.xtextbox.setText('{:6.2f}'.format(X[0]))
        self.ytextbox.setText('{:6.2f}'.format(X[1]))
        self.ztextbox.setText('{:6.2f}'.format(X[2]))

    def clear_figure(self):
        """
        This function clear the figure
        """
        self.axis.clear()

        # set the label for each axis
        self.axis.set_xlabel(self.axis_xlabel)
        self.axis.set_ylabel(self.axis_ylabel)
        self.axis.set_zlabel(self.axis_zlabel)
        # set the limit of each axis
        self.axis.set_xlim3d(self.axis_xlim3d[0], self.axis_xlim3d[1])
        self.axis.set_ylim3d(self.axis_ylim3d[0], self.axis_ylim3d[1])
        self.axis.set_zlim3d(self.axis_zlim3d[0], self.axis_zlim3d[1])

        return 0

    def draw_arm(self, X):
        """
        draw the arm in the figure
        :param X: [x,y,z] where x in the list of x coord of each joint of the arm, and so on.
        """
        [x, y, z] = X

        self.axis.plot(x, y, z, color='orange', linewidth = 3.0)
        self.axis.scatter(x, y, z, linewidth=3.0)

        return 0

    def draw_ref(self, list_ref):
        """
        draw on the figure a local reference frame
        :param ref: list of reference frame [0,x,y,z], vectors have a scale of 1
        """
        # scaling down the ref
        scale_x = (self.axis_xlim3d[1] - self.axis_xlim3d[0]) * 0.1
        scale_y = (self.axis_ylim3d[1] - self.axis_ylim3d[0]) * 0.1
        scale_z = (self.axis_zlim3d[1] - self.axis_zlim3d[0]) * 0.1

        print(list_ref[0])
        for i in range(len(list_ref)):

            pre_ref = list_ref[i].T
            ref = np.array([pre_ref[0], pre_ref[0] + pre_ref[1]*scale_x, pre_ref[0], pre_ref[0] + pre_ref[2]*scale_y, pre_ref[0], pre_ref[0] + pre_ref[3]*scale_z]).T[0:3]
            #print('ref {} :\n'.format(i), ref)
            print('T0 : \n', self.arm.kinematic.T0)
            self.axis.plot(ref[0], ref[1], ref[2], color='grey', linewidth = 2.0)
            self.axis.scatter(ref[0], ref[1], ref[2], color='grey', linewidth = 2.0)

