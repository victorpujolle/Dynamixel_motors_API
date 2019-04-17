import sys
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from Arm import Arm

from utils import *

import glob
import socket
import math


class robot_IK_interface(QtWidgets.QWidget):
    """
    The goal of this interface is the simulation of the IK
    Element of this interface will be incorporate later in an other interface for robot control
    """


    def __init__(self, arm):

        # super init
        super(robot_IK_interface, self).__init__()

        # link to the arm
        self.arm = arm

        # init UI
        self.init_UI()

        # init figure
        self.init_figure()

        # init menu
        self.init_menu()

    # ------INIT FUNCTIONS-----

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
        self.setGeometry(0, 0, 800, 600)
        self.FigureWidget.setGeometry(400, 0, 700, 600)
        self.MenuWidget.setGeometry(0, 0, 300, 600)

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

        self.axis_xlabel = "X"
        self.axis_ylabel = "Y"
        self.axis_zlabel = "Z"
        self.axis_xlim3d = [-0.4, 0.4]
        self.axis_ylim3d = [-0.4, 0.4]
        self.axis_zlim3d = [0, 0.8]

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
        # create label
        self.label_input= QtWidgets.QLabel(self)
        self.label_output = QtWidgets.QLabel(self)

        self.label_X  = QtWidgets.QLabel(self)
        self.label_Y  = QtWidgets.QLabel(self)
        self.label_Z  = QtWidgets.QLabel(self)
        self.label_aX = QtWidgets.QLabel(self)
        self.label_aY = QtWidgets.QLabel(self)
        self.label_aZ = QtWidgets.QLabel(self)

        self.label_q0 = QtWidgets.QLabel(self)
        self.label_q1 = QtWidgets.QLabel(self)
        self.label_q2 = QtWidgets.QLabel(self)
        self.label_q3 = QtWidgets.QLabel(self)
        self.label_q4 = QtWidgets.QLabel(self)
        self.label_q5 = QtWidgets.QLabel(self)

        # create textbox
        self.textbox_input_X  = QtWidgets.QLineEdit(self)
        self.textbox_input_Y  = QtWidgets.QLineEdit(self)
        self.textbox_input_Z  = QtWidgets.QLineEdit(self)
        self.textbox_input_aX = QtWidgets.QLineEdit(self)
        self.textbox_input_aY = QtWidgets.QLineEdit(self)
        self.textbox_input_aZ = QtWidgets.QLineEdit(self)

        self.textbox_output_q0 = QtWidgets.QLineEdit(self)
        self.textbox_output_q1 = QtWidgets.QLineEdit(self)
        self.textbox_output_q2 = QtWidgets.QLineEdit(self)
        self.textbox_output_q3 = QtWidgets.QLineEdit(self)
        self.textbox_output_q4 = QtWidgets.QLineEdit(self)
        self.textbox_output_q5 = QtWidgets.QLineEdit(self)

        # create button
        self.button_calculate = QtWidgets.QPushButton('calculate', self)
        self.button_draw_goal = QtWidgets.QPushButton('draw', self)
        self.button_draw = QtWidgets.QPushButton('draw', self)

        # resize button and textbox
        self.textbox_input_X.resize(100,20)
        self.textbox_input_Y.resize(100,20)
        self.textbox_input_Z.resize(100,20)
        self.textbox_input_aX.resize(100,20)
        self.textbox_input_aY.resize(100,20)
        self.textbox_input_aZ.resize(100,20)

        self.textbox_output_q0.resize(100,20)
        self.textbox_output_q1.resize(100,20)
        self.textbox_output_q2.resize(100,20)
        self.textbox_output_q3.resize(100,20)
        self.textbox_output_q4.resize(100,20)
        self.textbox_output_q5.resize(100,20)

        self.button_calculate.resize(100,30)
        self.button_draw_goal.resize(100,30)
        self.button_draw.resize(100,30)


        # set text of lables)
        self.label_input.setText('INPUT')
        self.label_output.setText('OUTPUT')

        self.label_X.setText('X :')
        self.label_Y.setText('Y :')
        self.label_Z.setText('Z :')
        self.label_aX.setText('aX :')
        self.label_aY.setText('aY :')
        self.label_aZ.setText('aZ :')

        self.label_q0.setText('q0 :')
        self.label_q1.setText('q1 :')
        self.label_q2.setText('q2 :')
        self.label_q3.setText('q3 :')
        self.label_q4.setText('q4 :')
        self.label_q5.setText('q5 :')

        # set location of everythink

        self.label_input.move(15+35, 10)
        self.label_output.move(215+35,10)

        self.label_X.move(15, 30)
        self.label_Y.move(15, 30 + 30)
        self.label_Z.move(15, 30 + 2 * 30)
        self.label_aX.move(15, 30 + 3 * 30)
        self.label_aY.move(15, 30 + 4 * 30)
        self.label_aZ.move(15, 30 + 5 * 30)

        self.textbox_input_X.move(15 + 35, 30)
        self.textbox_input_Y.move(15 + 35, 30 + 30)
        self.textbox_input_Z.move(15 + 35, 30 + 2 * 30)
        self.textbox_input_aX.move(15 + 35, 30 + 3 * 30)
        self.textbox_input_aY.move(15 + 35, 30 + 4 * 30)
        self.textbox_input_aZ.move(15 + 35, 30 + 5 * 30)

        self.label_q0.move(215, 30)
        self.label_q1.move(215, 30 + 30)
        self.label_q2.move(215, 30 + 2 * 30)
        self.label_q3.move(215, 30 + 3 * 30)
        self.label_q4.move(215, 30 + 4 * 30)
        self.label_q5.move(215, 30 + 5 * 30)

        self.textbox_output_q0.move(215 + 35, 30)
        self.textbox_output_q1.move(215 + 35, 30 + 30)
        self.textbox_output_q2.move(215 + 35, 30 + 2 * 30)
        self.textbox_output_q3.move(215 + 35, 30 + 3 * 30)
        self.textbox_output_q4.move(215 + 35, 30 + 4 * 30)
        self.textbox_output_q5.move(215 + 35, 30 + 5 * 30)

        self.button_calculate.move(150, 220)
        self.button_draw_goal.move(15+35, 220)
        self.button_draw.move(215+35, 220)


        return 0

    # -----BUTTON FUNCTIONS-----

    def on_draw_click(self):
        """
        method linked to the button draw
        """
        print('--- EVENT : DRAW CLICK ---')
        q_deg = np.array(self.get_simu_input()).astype(float)
        q_rad = deg2rad(q_deg)
        print('q [deg] :', q_deg)
        print('q [rad] :', q_rad)

        # figure and UI control
        R, t = self.arm.kinematic.compute_FK(q_rad)  # compute the total kinematics

        x, y, z = self.arm.kinematic.compute_pose(q_rad)  # compute all partial kinematic

        self.clear_figure()  # clear figure
        self.draw_arm([x, y, z])  # draw the arm
        # self.draw_ref(list_vector_frame, list_origin_frame)
        self.FigureCanvas.draw()

        return 0

    def on_readpos_click(self):
        """
        method read the postion of the robot and set the textboxes
        """
        print('--- EVENT : READ CLICK ---')
        self.arm.read_arm_postion()
        pos = self.arm.joint_angles
        print(pos)

        self.robotread0_textbox.setText('{:6.2f}'.format(pos[0]))
        self.robotread1_textbox.setText('{:6.2f}'.format(pos[1]))
        self.robotread2_textbox.setText('{:6.2f}'.format(pos[2]))
        self.robotread3_textbox.setText('{:6.2f}'.format(pos[3]))
        self.robotread4_textbox.setText('{:6.2f}'.format(pos[4]))
        self.robotread5_textbox.setText('{:6.2f}'.format(pos[5]))

        return 0

    def on_move_click(self):
        """
        method linked to the button move
        """
        print('--- EVENT : MOVE CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_move_input()
        X = [th0, th1, th2, th3, th4, th5]
        angles = [float(X[i]) if (X[i] != '') else X[i] for i in range(len(X))]
        self.arm.set_arm_position(angles)
        return 0

    def on_transfert_button_right(self):
        """
        method linked to the button transfert =>
        """
        print('--- EVENT : TRANSFERT => CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_simu_input()
        angles = [th0, th1, th2, th3, th4, th5]
        print(angles)
        self.set_move_input(angles)
        return 0

    def on_transfert_button_left(self):
        """
        method linked to the button transfert <=
        """
        print('--- EVENT : TRANSFERT <= CLICK ---')
        th0, th1, th2, th3, th4, th5 = self.get_move_input()
        angles = [th0, th1, th2, th3, th4, th5]
        print(angles)
        self.set_simu_input(angles)
        return 0

    # -----GETTER AND SETTER-----

    def get_simu_input(self):
        """
        getter of the input values
        :return: return the input values
        """
        th0 = self.simu0_textbox.text()
        th1 = self.simu1_textbox.text()
        th2 = self.simu2_textbox.text()
        th3 = self.simu3_textbox.text()
        th4 = self.simu4_textbox.text()
        th5 = self.simu5_textbox.text()

        if th0 == '':
            th0 = '0'
            self.simu0_textbox.setText('0')

        if th1 == '':
            th1 = '0'
            self.simu1_textbox.setText('0')

        if th2 == '':
            th2 = '0'
            self.simu2_textbox.setText('0')

        if th3 == '':
            th3 = '0'
            self.simu3_textbox.setText('0')

        if th4 == '':
            th4 = '0'
            self.simu4_textbox.setText('0')

        if th5 == '':
            th5 = '0'
            self.simu5_textbox.setText('0')

        return th0, th1, th2, th3, th4, th5

    def get_move_input(self):
        """
        getter of the input values
        :return: return the input values
        """
        th0 = self.robotmove0_textbox.text()
        th1 = self.robotmove1_textbox.text()
        th2 = self.robotmove2_textbox.text()
        th3 = self.robotmove3_textbox.text()
        th4 = self.robotmove4_textbox.text()
        th5 = self.robotmove5_textbox.text()

        return th0, th1, th2, th3, th4, th5

    def set_move_input(self, X):
        """
        method read the postion of the robot and set the textboxes
        """

        self.robotmove0_textbox.setText(X[0])
        self.robotmove1_textbox.setText(X[1])
        self.robotmove2_textbox.setText(X[2])
        self.robotmove3_textbox.setText(X[3])
        self.robotmove4_textbox.setText(X[4])
        self.robotmove5_textbox.setText(X[5])

        return 0

    def set_simu_input(self, X):
        """
        method read the postion of the robot and set the textboxes
        """

        self.simu0_textbox.setText(X[0])
        self.simu1_textbox.setText(X[1])
        self.simu2_textbox.setText(X[2])
        self.simu3_textbox.setText(X[3])
        self.simu4_textbox.setText(X[4])
        self.simu5_textbox.setText(X[5])

        return 0

    # -----DRAWING FUNCTIONS-----

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

        self.axis.plot(x, y, z, color='orange', linewidth=3.0)
        self.axis.scatter(x, y, z, linewidth=3.0)

        return 0
