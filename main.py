import sys
import os

from DXSerialAPI import *
from Arm import *

from PyQt5 import QtWidgets,QtCore
from robot_setting_interface import robot_setting_interface
from robot_IK_interface import robot_IK_interface


if __name__ == '__main__':

    #----------------------------------- SERIAL PARAMETERS SETTING -----------------------------------
    # Creation of the motor API class (here only the definition of the parent attributes)
    # open COM3, baudrate 1000000
    #PORT_NAME = 'COM3'
    PORT_NAME = None
    BAUDRATE = 1000000
    TIME_OUT = 0.05
    #motorsAPI = DXSerialAPI(PORT_NAME, BAUDRATE, TIMEOUT=TIME_OUT)
    # -------------------------------------------------------------------------------------------------

    # ----------------------------------- ROBOT PARAMETERS SETTING ------------------------------------
    #Creation of the Arm API class
    joint_number = 6
    motor_number = 7
    motors_id = [0,1,2,3,4,5,6]
    ArmAPI = Arm(PORT_NAME, BAUDRATE, TIME_OUT=TIME_OUT, joint_number=joint_number, motor_number=motor_number, motors_id=motors_id)
    # -------------------------------------------------------------------------------------------------

    # --------------------------------------- MOUVEMENTS TESTS ----------------------------------------
    UI = True

    if UI:
        # UI
        QApp = QtWidgets.QApplication(sys.argv)
        app = robot_IK_interface(ArmAPI)
        app.show()
        sys.exit(QApp.exec_())

    else:
        # eval
        d_list= []
        for i in range(100):
            print('i :',i)
            goal = np.array([(np.random.rand()-0.5)*0.4 , (np.random.rand()-0.5)*0.4 , (np.random.rand()*0.2+0.4)  , 0 , 0 , 0])
            first_guess = np.array([0 , 0 , 0 , 0 , 0 , 0])
            res = ArmAPI.IK.gradient_descent(goal, first_guess)
            d_list.append(ArmAPI.IK.norm_eval(goal, res['x']))

        d_list = np.array(d_list)
        np.savetxt('../Evaluation_files/result_optimization_100.csv', d_list, delimiter=',')



    # -------------------------------------------------------------------------------------------------





