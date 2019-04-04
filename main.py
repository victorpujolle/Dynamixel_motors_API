import sys
import os

from DXSerialAPI import *
from Arm import *

from PyQt5 import QtWidgets,QtCore
from control_interface_1 import Application


if __name__ == '__main__':

    #----------------------------------- SERIAL PARAMETERS SETTING -----------------------------------
    # Creation of the motor API class (here only the definition of the parent attributes)
    # open COM3, baudrate 1000000
    PORT_NAME = 'COM3'
    BAUDRATE = 1000000
    TIME_OUT = 0.05
    #motorsAPI = DXSerialAPI(PORT_NAME, BAUDRATE, TIMEOUT=TIME_OUT)
    # -------------------------------------------------------------------------------------------------

    # ----------------------------------- ROBOT PARAMETERS SETTING ------------------------------------
    #Creation of the Arm API class
    joint_number = 6
    motor_number = 7
    motors_id = [0,1,2,3,4,5,6]
    joints2motors = [[0],[1,2],[3],[4],[5],[6]]
    ArmAPI = Arm(PORT_NAME, BAUDRATE, TIME_OUT=TIME_OUT, joint_number=joint_number, motor_number=motor_number, motors_id=motors_id, joints2motors=joints2motors)
    # -------------------------------------------------------------------------------------------------

    # ----------------------------------------- MOTOR TESTING -----------------------------------------
    # motors existence
    all_motors_ok, details, details_msg = ArmAPI.test_motors()
    print('--- Are all the motors connected ? :', all_motors_ok, '\n--- Motors details :', details)
    #print(details_msg)
    # -------------------------------------------------------------------------------------------------

    # --------------------------------------- MOUVEMENTS TESTS ----------------------------------------
    pi = np.pi
    pi2 = pi/2

    q = [pi2,pi2,pi2,pi2,pi2,pi2]

    Gen = Generator(ArmAPI.LINKS_LENGTH,-90,90)


    # UI
    QApp = QtWidgets.QApplication(sys.argv)
    app = Application(ArmAPI)
    app.show()
    sys.exit(QApp.exec_())

    # -------------------------------------------------------------------------------------------------





