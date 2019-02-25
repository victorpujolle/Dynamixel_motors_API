from DXSerialAPI import DXSerialAPI
from utils import *

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

class Arm(DXSerialAPI):
    """
    This class represent my arm and can be used to control it
    """


    def __init__(self, PORT_NAME, BAUDRATE, TIME_OUT=0.001, joint_number=6, motor_number=6, motors_id=None, joints2motors=None):

        # __init__ of the parent class
        super(Arm, self).__init__(PORT_NAME, BAUDRATE, TIME_OUT=TIME_OUT)

        # general properties of the arm
        self.joint_number = joint_number # number of joint of the arm
        self.motor_number = motor_number # number of motor of the arm
        self.motors_id = motors_id # i :  the id of the motors, motors_id : real id of the device (in term of serial communication)
        self.joints2motors = joints2motors # i : the id of the joint, joints2motors[i] : the motor(s) linked to the joints

        # motors values that have to be initialized by reading the state of each motors in the - initialisation - function
        self.motors_angle_limits_byte = [] # the i-th element this list will contains [angle_clockwise_limit , angle_counterclockwise_limit] of the i-th motor (byte units used)
        self.motors_torque_limits_byte = [] # the i-th element this list will contains [max torque] of the i-th motor (byte units used)
        self.motors_angles_byte = [] # the angles of each motors (byte units used)
        self.motors_speed_byte = [] # the speed of each motors (byte unit used) (the speed of the motors has to be initialized first)

        # class constant init
        self.ANGLES_INIT = np.array([250, 100, 140, 220, 145, 220]) # angles initialisation of each JOINTS (not motors!!!!)
        self.DX_SPEED = 100 # speed of the motors
        self.LINKS_LENGTH = [0.045, 0.11, 0.04, 0.04, 0.11, 0.17] # links length

        # usefull flags
        self.flag_is_position_init = False # become true when the position of each joint has been initialize in set_arm_position (or initialisation_position)

        # arm values, this values need to be setted for any kinematics computation
        self.joint_position = np.zeros((self.joint_number)) # position of each joint

        # variables used for kinematics computation
        self.T_list = np.zeros((self.joint_number, 4, 4)) # list of the transfert matrix of each joint
        self.T_total = np.eye(4) # the total transfert matrix of the arm
        self.J = np.zeros((3,4))       # Jacobian
        self.JJt = np.zeros((3,3))     # J * transpose(J)
        self.invJJt = np.zeros((3,3))  # inverse of JJt
        self.JinvJJt = np.zeros((3,4)) # J*invJJt



    # ------------- READING -------------

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
        This function read the angle limits, the torque limit, the speed and the position of each motors
        """

        for motor in self.motors_id:
            self.motors_angle_limits_byte.append([self.read_angle_limit_clockwise_byte(motor), self.read_angle_limit_counterclockwise_byte(motor)])
            self.motors_torque_limits_byte.append(self.read_max_torque_limit_byte(motor))
            self.motors_angles_byte.append(self.read_present_position_byte(motor))
            self.motors_speed_byte.append(self.read_moving_speed_byte(motor))

        return 0

    # -----------------------------------

    # ------------- INITIALIZATION -------------

    def initialisation_speed(self, init_speed=None):
        """
        This function initialize the rotation speed of all the motor
        :param init_speed: the speed you want (in rmp) (if None, the class default speed will be taken)
        """
        speed = self.DX_SPEED if init_speed == None else init_speed
        self.set_moving_speed(0xFE, speed) # 0xFE is the broadcast ID, all the motors will read the signal

        return 0

    def is_angle_valid(self, motor_id, angle, error_raising=True):
        """
        The functions tests if the angle is a valid goal postion value of the motor, this function will also raise an error if the mode error_raising is set to true
        :param motor_id: id of the motor
        :param angle: angle (degree) that you want to test
        :param error_raising : true or false depending if you want to raise an error or not
        :return: True if the angle given is a valid value, False otherwise
        """
        if angle > self.motors_angle_limits_byte[motor_id][0]*self.ANGLE_UNIT or angle < self.motors_angle_limits_byte[motor_id][1]*self.ANGLE_UNIT:
            return True

        else:
            if error_raising:
                raise ValueError('The angle value {} have to be between the limit values [{} , {}] of the motor {}'.format(angle, self.motors_torque_limits_byte[0]*self.ANGLE_UNIT,self.motors_torque_limits_byte[1]*self.ANGLE_UNIT, motor_id))
            return False

    def write_single_joint_position(self, joint_id, angle):
        """
        :param joint_id: the id of the JOINT
        :param angle: the angle you want for the joint
        depending of the configuration of your robot, you have to rewrite this function
        """

        if joint_id == 0:
            if self.is_angle_valid(0, angle, error_raising=True):
                self.set_goal_position(0, angle)

        if joint_id == 1:
            if self.is_angle_valid(1, angle, error_raising=True) and self.is_angle_valid(2, 300 - angle, error_raising=True):
                self.set_goal_position(1, angle)
                self.set_goal_position(2, 300 - angle)

        if joint_id == 2:
            if self.is_angle_valid(3, angle, error_raising=True):
                self.set_goal_position(3, angle)

        if joint_id == 3:
            if self.is_angle_valid(4, angle, error_raising=True):
                self.set_goal_position(4, angle)

        if joint_id == 4:
            if self.is_angle_valid(5, angle, error_raising=True):
                self.set_goal_position(5, angle)

        if joint_id == 5:
            if self.is_angle_valid(6, angle, error_raising=True):
                self.set_goal_position(6, angle)

        if joint_id == 6:
            if self.is_angle_valid(7, angle, error_raising=True):
                self.set_goal_position(7, angle)

        return 0

    def initialisation_position(self, init_angles=None):
        """
        This function initialize the angle of each JOINT (this is actualy tricky if you have multiple motors for the same joint, this if why you should
        REWRITE THE UGLY FUNCTION _write_single_joint_position WITH ALL THE IF STATEMENTS for your robot (sorry but this is the more robust and simple way to sovle this problem)
        :param init_angles: init_angles[i] is the angle (in degree) of the i-th joint
        """

        angles = self.ANGLES_INIT if init_angles == None else init_angles

        self.set_arm_position(angles)

        return 0

    # -----------------------------------

    # ------------- CONTROLS -------------

    def set_arm_position(self, angles):
        """
        This function sets the position of all the joints
        :param angles: the goal position
        """
        if len(angles) != self.joint_number:
            raise ValueError('The length of the input angles : {} list should be the number of joint {} '.format(len(angles),self.joint_number))

        for joint in range(self.joint_number):
            self.write_single_joint_position(joint, angles[joint])
            self.joint_position[joint] = angles[joint]

        self.flag_is_position_init = True # the position is now setted

        return 0

    # ------------------------------------

    # ------------- FORWARD KINEMATICS -------------

    def calculate_T(self, d, r, alpha, theta):
        """
        calculate the Denavit and Hartenberg matrix for a joint
        :param d: link offset
        :param r: link length
        :param alpha: link twist
        :param theta: link angle
        :return: the Denavit and Hartenberg matrix
        """
        T = np.zeros((4,4))

        ct, st, ca, sa = 0, 0, 0, 0

        if  round(theta, 5) == 0: # zero case
            ct = 1
            st = 0
        elif round(theta, 5) == round(np.pi / 2, 5): # pi/2 case
            ct = 0
            st = 1
        elif round(theta, 5) == round(np.pi, 5):  # pi case
            ct = -1
            st = 0
        elif round(theta, 5) == round(3 * np.pi / 2, 5):  # 3pi/2 case
            ct = 0
            st = -1
        else: # general case
            ct = np.cos(theta)
            st = np.sin(theta)

        if round(alpha, 5) == 0: # zero case
            ca = 1
            sa = 0
        elif round(alpha, 5) == round(np.pi/2, 5): # pi/2 case
            ca = 0
            sa = 1
        elif round(alpha, 5) == round(np.pi, 5):  # pi case
            ca = -1
            sa = 0
        elif round(alpha, 5) == round(3 * np.pi / 2, 5):  # 3pi/2 case
            ca = 0
            sa = -1
        else: # general case
            ca = np.cos(alpha)
            sa = np.sin(alpha)

        # rotation, first line
        T[0, 0] = ct
        T[0, 1] = - st * ca
        T[0, 2] = st * sa

        # rotation second line
        T[1, 0] = st
        T[1, 1] = ct * ca
        T[1, 2] = - ct * sa

        # rotation last line
        T[2, 1] = sa
        T[2, 2] = ca

        # translation
        T[0, 3] = r * ct
        T[1, 3] = r * st
        T[2, 3] = d

        # last value
        T[3, 3] = 1

        return T

    def calculate_forward_kinematics_matrix(self):
        """
        This function calculates the complete forward kinematics matrix of the arm
        :return: T the total kinematics matrix = T1 * T2 * ... * Tn
        """
        if not(self.flag_is_position_init):
            raise ValueError('The postion has not been initialized, you need to initialize the position with set_arm_position or initialisation_position before computing kinematics')

        for joint in range(self.joint_number):
            theta = self.joint_position[joint] * 360 / (2*np.pi)
            self.T_list[joint] = self.calculate_T(0, self.LINKS_LENGTH[joint], np.pi/2, theta)
            self.T_total = self.T_total.dot(self.T_list[joint])


        print(self.T_total)

    def calcutate_resultant_vector(self, P, T=None):
        """
        This function computes the dot product T.P, which gives this vector in the referenciel of the effector
        :param P: vector
        :param T: the transformation matrix
        :return: P but transformed by the robot
        """
        T = self.T_total if T is None else T # T default value is self.T_total

        return T.dot(P)

    # -----------------------------------

    # ------------- 3D DISPLAY -------------

    def display_3d_robot(self):
        """
        this function displays a 3D representation of the robot's joints
        unit is the meter
        """
        fig = plt.figure()
        ax = plt.axes(projection='3d')

        origin = np.array([0, 0, 0, 1])
        origin_x = np.array([1, 0, 0, 1])
        origin_y = np.array([0, 1, 0, 1])
        origin_z = np.array([0, 0, 1, 1])
        origin_ref = np.array([origin_x, origin, origin_y, origin, origin_z]).T

        #ax.plot3D(origin_ref[0], origin_ref[1], origin_ref[2])

        T = np.eye(4)
        arm_origins = np.zeros((self.joint_number,4))
        arm_origins[:,3] = np.ones((self.joint_number))
        print(arm_origins)

        for joint in range(self.joint_number):

            T = self.T_list[joint] # transformation matrix joint -> joint+1






        plt.show()

    # -----------------------------------







    # TODO : forward cinematic function, backward cinematic solver


