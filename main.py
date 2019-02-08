from DXSerialAPI import *
from Arm import *


if __name__ == '__main__':

    #----------------------------------- SERIAL PARAMETERS SETTING -----------------------------------
    # Creation of the motor API class (here only the definition of the parent attributes)
    # open COM3, baudrate 1000000
    PORT_NAME = 'COM3'
    BAUDRATE = 1000000
    TIME_OUT = 0.1
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
    all_motors_ok, details = ArmAPI.test_motors()
    print('--- Are all the motors connected ? :', all_motors_ok, '\n--- Motors details :', details)
    # -------------------------------------------------------------------------------------------------

    # -------------------------------------- COMMUNICATION TESTS --------------------------------------
    #
    #voltage = ArmAPI.read_present_voltage(id)
    #temperature = ArmAPI.read_internal_temperature(id)
    #present_position = ArmAPI.read_present_position(id)
    #moving_speed = ArmAPI.read_moving_speed(id)
    #clock_angle_limit = ArmAPI.read_angle_limit_clockwise_byte(id)
    #counterclock_angle_limit = ArmAPI.read_angle_limit_counterclockwise_byte(id)
    #
    #print('voltage       :', voltage, 'V')
    #print('temperature   :', temperature, 'C')
    #print('position      :', present_position, '°')
    #print('moving speed  :', moving_speed, 'rmp')
    #print('limit angle - :', clock_angle_limit)
    #print('limit angle + :', counterclock_angle_limit)
    # -------------------------------------------------------------------------------------------------

    # ---------------------------------------- INITIALISATION -----------------------------------------
    print('--- INITIALISATION ---')
    ArmAPI.read_state()
    print('angles limit :', ArmAPI.motors_angle_limits_byte)
    print('torque limit :', ArmAPI.motors_torque_limits_byte)
    print('position :', ArmAPI.motors_angles_byte)
    # -------------------------------------------------------------------------------------------------

    # --------------------------------------- MOUVEMENTS  TESTS ---------------------------------------
    ArmAPI.initialisation_speed()
    ArmAPI.set_goal_position(1, 100)
    ArmAPI.set_goal_position(2,100)
    # -------------------------------------------------------------------------------------------------




