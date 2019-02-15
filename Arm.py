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
        self.motors_speed_byte = [] # the speed of each motors (byte unit used) (the speed of the motors has to be initialized first)

        # class constant init
        self.ANGLES_INIT = np.array([250, 100, 140, 220, 145, 220])  # angles initialisation of each JOINTS (not motors!!!!)
        self.DX_SPEED = 100  # speed of the motors

    def test_multiple_id(self, list_id):
        """
        this function will test all the id in list_id
        :param list_id: list of id to be tested
        :return: exist: boolean list, if the motor list_id[i] exists then exist[i] == True
        """
        exist = []
        for id in list_id:
            exists, _ = self._PING(id)
            exist.append(exists)

        return exist

    def test_motors(self):
        """
        this function test if all the motors that are supposed to be in the arm exist
        this function use the variable motors_id, be sure to correctly initialize this variable
        :return: all_motors_exists, details: all_motors_exists is True if all motors exists, False otherwise, details give you details about which motors exist or not
                 details_msg: the responces of each motors
        """
        details = []
        details_msg = []
        all_motors_exists = True
        if self.motors_id == None:
            raise ValueError('self.motors_id is None, please link you motors')
        else:
            for id in self.motors_id:
                exists, message = self._PING(id)
                details.append(exists)
                details_msg.append(message)
                if not(exists): all_motors_exists = False

        return all_motors_exists, details, details_msg

    def read_full_state(self):
        """
        This function initializes the state of each motors, with there constrains
        """

        for motor in self.motors_id:
            self.motors_angle_limits_byte.append([self.read_angle_limit_clockwise_byte(motor), self.read_angle_limit_counterclockwise_byte(motor)])
            self.motors_torque_limits_byte.append(self.read_max_torque_limit_byte(motor))
            self.motors_angles_byte.append(self.read_present_position_byte(motor))
            self.motors_speed_byte.append(self.read_moving_speed_byte(motor))

        return 0

    def initialisation_speed(self, init_speed=None):
        """
        This function initialize the rotation speed of all the motor
        :param init_speed: the speed you want (in rmp) (if None, the class default speed will be taken)
        """
        speed = self.DX_SPEED if init_speed == None else init_speed
        self.set_moving_speed(0xFE, speed) # 0xFE is the broadcast ID, all the motors will read the signal

        return 0

    def write_single_joint_position(self, joint_id, angle):
        """
        :param joint_id: the id of the JOINT
        :param angle: the angle you want for the joint
        depending of the configuration of your robot, you have to rewrite this function
        """
        if joint_id == 0: self.set_goal_position(0, angle)
        if joint_id == 1:
            self.set_goal_position(1, angle)
            self.set_goal_position(2, 300 - angle)

        if joint_id == 2: self.set_goal_position(3, angle)
        if joint_id == 3: self.set_goal_position(4, angle)
        if joint_id == 4: self.set_goal_position(5, angle)
        if joint_id == 5: self.set_goal_position(6, angle)
        if joint_id == 6: self.set_goal_position(7, angle)

    def initialisation_position(self, init_angles=None):
        """
        This function initialize the angle of each JOINT (this is actualy tricky if you have multiple motors for the same joint, this if why you should
        REWRITE THE UGLY FUNCTION _write_single_joint_position WITH ALL THE IF STATEMENTS for your robot (sorry but this is the more robust and simple way to sovle this problem)
        :param init_angles: init_angles[i] is the angle (in degree) of the i-th joint
        """

        angles = self.ANGLES_INIT if init_angles == None else init_angles

        for joint in range(self.joint_number):
            self.write_single_joint_position(joint, angles[joint])

        return 0


