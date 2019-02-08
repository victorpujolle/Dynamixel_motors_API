from DXSerialAPI import DXSerialAPI
from utils import *

class Arm(DXSerialAPI):
    """
    This class represent my arm and can be used to control it
    """


    def __init__(self, PORT_NAME, BAUDRATE, TIME_OUT=0.001, joint_number=6, motor_number=6, motors_id=None, joints2motors=None):

        # __init__ of the parent class
        super(Arm, self).__init__(PORT_NAME, BAUDRATE, TIME_OUT=TIME_OUT)

        self.joint_number = joint_number # number of joint of the arm
        self.motor_number = motor_number # number of motor of the arm
        self.motors_id = motors_id # i :  the id of the motors, motors_id : real id of the device (in term of serial communication)
        self.joints2motors = joints2motors # i : the id of the joint, joints2motors[i] : the motor(s) linked to the joints

        # values that have to be initialized by reading the state of each motors in the - initialisation - function
        self.motors_angle_limits_byte = [] # the i-th element this list will contains [angle_clockwise_limit , angle_counterclockwise_limit] of the i-th motor (byte units used)
        self.motors_torque_limits_byte = [] # the i-th element this list will contains [max torque] of the i-th motor (byte units used)
        self.motors_angles_byte = [] # the angles of each motors (byte units used)

        # class constant init
        self.ANGLES_INIT = np.array([150, 100, 0.0, 250, 0.0, 70])  # angles initialisation of each joints
        self.DX_SPEED = 100  # speed of the motors

    def test_multiple_id(self, list_id):
        """
        this function will test all the id in list_id
        :param list_id: list of id to be tested
        :return: exist: boolean list, if the motor list_id[i] exists then exist[i] == True
        """
        exist = []
        for id in list_id:
            exists, message = self._PING(id)
            exist.append(exists)

        return exist

    def test_motors(self):
        """
        this function test if all the motors that are supposed to be in the arm exist
        this function use the variable motors_id, be sure to correctly initialize this variable
        :return: all_motors_exists, details: all_motors_exists is True if all motors exists, False otherwise, details give you details about which motors exist or not
        """
        details = []
        all_motors_exists = True
        if self.motors_id == None:
            raise ValueError('self.motors_id is None, please link you motors')
        else:
            for id in self.motors_id:
                exists, message = self._PING(id)
                details.append(exists)
                if not(exists): all_motors_exists = False

        return all_motors_exists, details

    def read_state(self):
        """
        This function initializes the state of each motors, with there constrains
        """

        for motor in self.motors_id:
            self.motors_angle_limits_byte.append([self.read_angle_limit_clockwise_byte(motor), self.read_angle_limit_counterclockwise_byte(motor)])
            self.motors_torque_limits_byte.append(self.read_max_torque_limit_byte(motor))
            self.motors_angles_byte.append(self.read_present_position_byte(motor))

        return 0

    def initialisation_speed(self, init_speed=None):
        """
        This function initialize the rotation speed of all the motor
        :param init_speed: the speed you want (in rmp) (if None, the class default speed will be taken)
        """
        speed = self.DX_SPEED if init_speed == None else init_speed

        for motor in self.motors_id:
            self.set_moving_speed(motor, speed)

        return 0

    def initialisation_position(self, init_angles=None):
        """
        This function initialize the anlge of each JOINT (this is actualy tricky if you have multiple motors for the same joint, this if why you should
        REWRITE THE UGLY SUB FUNCTION WITH ALL THE IF STATEMENTS for your robot (sorry but this is the more robust and simple way to sovle this problem)
        :param init_angles: init_angles[i] is the angle (in degree) of the i-th joint
        """
        def write_single_joint_position(joint_id, angle):
            # yes you have to rewrite this functions for your robot ;)
            if joint_id == 0: self.set_goal_position(0, angle)
            if joint_id == 1:
                self.set_goal_position(1, angle)
                self.set_goal_position(2, 300-angle)

            if joint_id == 2: self.set_goal_position(3, angle)
            if joint_id == 3: self.set_goal_position(4, angle)
            if joint_id == 4: self.set_goal_position(5, angle)
            if joint_id == 5: self.set_goal_position(6, angle)
            if joint_id == 6: self.set_goal_position(7, angle)

        angles = self.ANGLES_INIT if init_angles == None else init_angles

        for joint in self.joint_number:
            write_single_joint_position(joint, angles[joint])

        return 0


