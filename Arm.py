import time

from DXSerialAPI import DXSerialAPI
from utils import *
from kinematic_generator import Generator

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
        self.ANGLES_INIT = np.array([60, 100, 140, 220, 145, 220]) # angles initialisation of each JOINTS (not motors!!!!)
        self.DX_SPEED = 100 # speed of the motors
        self.LINKS_LENGTH = [0.045, 0.11, 0.04, 0.04, 0.11, 0.13] # links length
        self.ANGLE_OFFSET = np.array([150, 132, 150, 150, 150, 150]) # angle offset used to calculate DH parameters (because the angle origin of the motors is not the same that the angle origin of the DH param)

        # usefull flags
        self.flag_is_position_init = False # become true when the position of each joint has been initialize in set_arm_position (or initialize_position)
        self.flag_is_DH_param_init = False # become true when the DH parameters of each joint has been initialize in set_arm_position (or initialize_position)

        # arm values, this values need to be setted for any kinematics computation
        self.joint_position = np.zeros((self.joint_number)) # position of each joint

        # variables used for kinematics computation
        self.DH_parameters = np.zeros((self.joint_number,4)) # DH param : d, theta,r, alpha
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

    def read_arm_postion(self):
        """
        This function read the position of each motors
        """

        self.motors_angles = []

        for motor in self.motors_id:
            self.motors_angles.append(round(self.read_present_position_byte(motor) * self.ANGLE_UNIT,2))

        #print(self.motors_angles)

        joint_angles = self.motors_angles.copy()
        joint_angles.pop(2)
        self.joint_angles = np.array(joint_angles) - self.ANGLE_OFFSET

        print(self.joint_angles)

        return 0

    # -----------------------------------

    # ------------- INITIALIZATION -------------

    def initialize_speed(self, init_speed=None):
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
        if angle > 90 or angle < 90:
            return True

        else:
            if error_raising:
                raise ValueError('The angle value ({} given) have to be between the limit values [90 , 90] of the motor {}'.format(angle))
            return False

    def initialize_position(self, init_angles=None):
        """
        This function initialize the angle of each JOINT (this is actualy tricky if you have multiple motors for the same joint, this if why you should
        REWRITE THE UGLY FUNCTION _write_single_joint_position WITH ALL THE IF STATEMENTS for your robot (sorry but this is the more robust and simple way to sovle this problem)
        :param init_angles: init_angles[i] is the angle (in degree) of the i-th joint
        """

        angles = self.ANGLES_INIT if init_angles == None else init_angles

        self.set_arm_position(angles)

        return 0

    def initialize_DH_param(self, DH_param=None):
        """
        Initialisation of the DH parameters of the arm
        :param DH_param: if you want to specify of value to this function, if None the default values will be used (modify the function to adapt it to your arm)
        """
        if DH_param is not None:
            self.DH_parameters = np.copy(DH_param)
        else:
            if not (self.flag_is_position_init):
                raise ValueError('The position has not been initialized, you need to initialize the position with set_arm_position or initialize_position before computing DH parameters')

            self.DH_parameters = np.array([
                [self.LINKS_LENGTH[0], (self.joint_position[0] + self.ANGLE_OFFSET_BYTE[0] * self.ANGLE_UNIT) / (360 / (2 * np.pi)), 0                   , 0],
                [0                   , (self.joint_position[1] + self.ANGLE_OFFSET_BYTE[1] * self.ANGLE_UNIT) / (360 / (2 * np.pi)), self.LINKS_LENGTH[1], np.pi / 2],
                [self.LINKS_LENGTH[2], (self.joint_position[2] + self.ANGLE_OFFSET_BYTE[2] * self.ANGLE_UNIT) / (360 / (2 * np.pi)), 0                   , 0],
                [0                   , (self.joint_position[3] + self.ANGLE_OFFSET_BYTE[3] * self.ANGLE_UNIT) / (360 / (2 * np.pi)), self.LINKS_LENGTH[3], np.pi / 2],
                [self.LINKS_LENGTH[4], (self.joint_position[4] + self.ANGLE_OFFSET_BYTE[4] * self.ANGLE_UNIT) / (360 / (2 * np.pi)), 0                   , 0],
                [0                   , (self.joint_position[5] + self.ANGLE_OFFSET_BYTE[5] * self.ANGLE_UNIT) / (360 / (2 * np.pi)), self.LINKS_LENGTH[5], np.pi / 2],
            ])
        self.flag_is_DH_param_init = True
        return 0

    # -----------------------------------

    # ------------- CONTROLS -------------

    def write_single_joint_position(self, joint_id, angle):
        """
        :param joint_id: the id of the JOINT
        :param angle: the angle you want for the joint, if angle is '' it does nothing
        depending of the configuration of your robot, you have to rewrite this function
        """

        angle += self.ANGLE_OFFSET[joint_id]

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

    def set_arm_position(self, angles):
        """
        This function sets the position of all the joints
        :param angles: the goal position
        """
        if len(angles) != self.joint_number:
            raise ValueError('The length of the input angles : the input list has a length of {} and  this length should be the number of joint {} '.format(len(angles),self.joint_number))

        for joint in range(self.joint_number):
            if angles[joint] != '':
                self.write_single_joint_position(joint, angles[joint])
                self.joint_position[joint] = angles[joint]

        self.flag_is_position_init = True # the position is now setted

        return 0

    def test_angle_limits(self):
        """
        this function will move each joints of the robot, one by one, from the min angle to the max angle
        """

        def move_joint_from_min_to_max(joint_id, min, max):
            self.write_single_joint_position(joint_id,min)
            time.sleep(3)
            self.write_single_joint_position(joint_id, max)
            time.sleep(3)
            self.write_single_joint_position(joint_id, self.ANGLES_INIT[joint_id])
            time.sleep(1)


        min0 = self.motors_angle_limits_byte[0][0] * self.ANGLE_UNIT
        max0 = self.motors_angle_limits_byte[0][1] * self.ANGLE_UNIT
        print(min0,max0)
        move_joint_from_min_to_max(0,min0,max0)

        #min1 = self.motors_angle_limits_byte[1][0] * self.ANGLE_UNIT
        #max1 = self.motors_angle_limits_byte[1][1] * self.ANGLE_UNIT
        #print(min1, max1)
        #move_joint_from_min_to_max(1, min1, max1)

        min2 = self.motors_angle_limits_byte[2][0] * self.ANGLE_UNIT
        max2 = self.motors_angle_limits_byte[2][1] * self.ANGLE_UNIT
        print(min2, max2)
        move_joint_from_min_to_max(2, min2, max2)

        min3 = self.motors_angle_limits_byte[3][0] * self.ANGLE_UNIT
        max3 = self.motors_angle_limits_byte[3][1] * self.ANGLE_UNIT
        print(min3, max3)
        move_joint_from_min_to_max(3, min3, max3)

        min4 = self.motors_angle_limits_byte[4][0] * self.ANGLE_UNIT
        max4 = self.motors_angle_limits_byte[4][1] * self.ANGLE_UNIT
        print(min4, max4)
        move_joint_from_min_to_max(4, min4, max4)

        min5 = self.motors_angle_limits_byte[5][0] * self.ANGLE_UNIT
        max5 = self.motors_angle_limits_byte[5][1] * self.ANGLE_UNIT
        print(min5, max5)
        move_joint_from_min_to_max(5, min5, max5)

        min6 = self.motors_angle_limits_byte[6][0] * self.ANGLE_UNIT
        max6 = self.motors_angle_limits_byte[6][1] * self.ANGLE_UNIT
        print(min6, max6)
        move_joint_from_min_to_max(6, min6, max6)

    # ------------------------------------

    # ------------- FORWARD KINEMATICS -------------

    def calculate_T(self, d, theta, r, alpha):
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

    def calculate_forward_kinematics_matrices(self):
        """
        This function calculates the complete forward kinematics matrix of the arm
        :return: T the total kinematics matrix = T1 * T2 * ... * Tn
        """
        if not(self.flag_is_DH_param_init):
            raise ValueError('The DH parameters have not been initialized, you need to initialize the DH parameters with initialze_DH_param before computing Denavit–Hartenberg matrices')

        for joint in range(self.joint_number):
            self.T_list[joint] = self.calculate_T(self.DH_parameters[joint][0],self.DH_parameters[joint][1],self.DH_parameters[joint][2],self.DH_parameters[joint][3])
            self.T_total = self.T_total.dot(self.T_list[joint])

        T0 = np.eye(4)
        T1 = np.eye(4)
        T2 = np.eye(4)
        T3 = np.eye(4)
        T4 = np.eye(4)
        T5 = np.eye(4)

        theta1 = q[0]
        theta2 = q[1]
        theta3 = q[2]
        theta4 = q[3]
        theta5 = q[4]
        theta6 = q[5]

        L1 = self.L[0]
        L2 = self.L[1]
        L3 = self.L[2]
        L4 = self.L[3]
        L5 = self.L[4]
        L6 = self.L[5]

        theta1 = 0
        theta2 = 0
        theta3 = 0
        theta4 = 0
        theta5 = 0
        theta6 = 0

        print('joint 0 en degree :', theta1 * (360 / (2 * np.pi)))
        print('joint 1 en degree :', theta2 * (360 / (2 * np.pi)))
        print('joint 2 en degree :', theta3 * (360 / (2 * np.pi)))
        print('joint 3 en degree :', theta4 * (360 / (2 * np.pi)))
        print('joint 4 en degree :', theta5 * (360 / (2 * np.pi)))
        print('joint 5 en degree :', theta6 * (360 / (2 * np.pi)))

        print('L0 =',self.LINKS_LENGTH[0] ,'m')
        print('L1 =',self.LINKS_LENGTH[1] ,'m')
        print('L2 =',self.LINKS_LENGTH[2] ,'m')
        print('L3 =',self.LINKS_LENGTH[3] ,'m')
        print('L4 =',self.LINKS_LENGTH[4] ,'m')
        print('L5 =',self.LINKS_LENGTH[5] ,'m')


        ct1 = np.cos(theta1)
        ct2 = np.cos(theta2)
        ct3 = np.cos(theta3)
        ct4 = np.cos(theta4)
        ct5 = np.cos(theta5)
        ct6 = np.cos(theta6)

        st1 = np.sin(theta1)
        st2 = np.sin(theta2)
        st3 = np.sin(theta3)
        st4 = np.sin(theta4)
        st5 = np.sin(theta5)
        st6 = np.sin(theta6)

        L1 = self.LINKS_LENGTH[0]
        L2 = self.LINKS_LENGTH[1]
        L3 = self.LINKS_LENGTH[2]
        L4 = self.LINKS_LENGTH[3]
        L5 = self.LINKS_LENGTH[4]
        L6 = self.LINKS_LENGTH[5]

        T0[0, 0] = ct1
        T0[0, 1] =-st1
        T0[1, 0] = st1
        T0[1, 1] = ct1

        T1[0, 0] = ct2
        T1[0, 2] = st2
        T1[2, 0] =-st2
        T1[2, 2] = ct2

        T2[1, 1] = ct3
        T2[1, 2] =-st3
        T2[2, 1] = st3
        T2[2, 2] = ct3

        T3[0, 0] = ct4
        T3[0, 2] = st4
        T3[2, 0] =-st4
        T3[2, 2] = ct4

        T4[1, 1] = ct5
        T4[1, 2] =-st5
        T4[2, 1] = st5
        T4[2, 2] = ct5

        T5[0, 0] = ct6
        T5[0, 2] = st6
        T5[2, 0] =-st6
        T5[2, 2] = ct6

        T0[0, 3] = 0
        T0[1, 3] = 0
        T0[2, 3] = L1

        T1[0, 3] = L2 * ct2
        T1[1, 3] = 0
        T1[2, 3] = L2 * st2

        T2[0, 3] = L3
        T2[1, 3] = 0
        T2[2, 3] = 0

        T3[0, 3] = L4 * ct4
        T3[1, 3] = 0
        T3[2, 3] = L4 * st4

        T4[0, 3] = L5
        T4[1, 3] = 0
        T4[2, 3] = 0

        T5[0, 3] = L6 * ct6
        T5[1, 3] = 0
        T5[2, 3] = L6 * st6

        print('T0 =\n',T0)
        print('T1 =\n',T1)
        print('T2 =\n',T2)
        print('T3 =\n',T3)
        print('T4 =\n',T4)
        print('T5 =\n',T5)

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

        ax.plot3D(origin_ref[0], origin_ref[1], origin_ref[2])
        T = np.eye(4)
        arm_origins = [origin]
        ref = [origin_ref]

        for joint in range(self.joint_number):
            T = T.dot(self.T_list_hard[joint])
            print('T', joint+1, ':\n',np.round(self.T_list_hard[joint],2))

            new_origin = T.dot(origin)
            arm_origins.append(new_origin)

            new_origin_x = T.dot(origin_x)
            new_origin_y = T.dot(origin_y)
            new_origin_z = T.dot(origin_z)

            new_origin_ref = np.array([new_origin_x, new_origin, new_origin_y, new_origin, new_origin_z]).T
            ref.append(new_origin_ref)


        arm_origins = np.array(arm_origins).T
        print('arm joints :\n',arm_origins.T)

        ax.plot3D(arm_origins[0], arm_origins[1], arm_origins[2])
        ax.scatter3D(arm_origins[0], arm_origins[1], arm_origins[2])

        #for r in ref:
        #    ax.plot3D(r[0],r[1],r[2])

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_zlim(0, 5)


        plt.show()

    # -----------------------------------







    # TODO : forward cinematic function, backward cinematic solver


