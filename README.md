# Dynamixel API

## Introduction
This repo provides an API for dynamixel motors written in Python. This can be useful 
for those who does not want to use the bugged and badly commented API provided by
dynamixel or those who do not want to translate all there documentation for japanese to english.
It also provide a API for controlling a robot made with this motors. 


## Description
This API comes with 2 classes :
- DXSerailAPI which is the low level communication API.
- ARM which inherits from DXSerialAPI and is here for the control of all your robot.
- kinematics_generator makes all the computation of the FK and IK
- the differents ._interface.py are interface used to control the robot, both the simulation and the real robot

There come two others files :
- utils.py which is for some utils functions that do not really feet in the classes.
- main.py which is obviously the main that should be executed.

## Dependencies
- Numpy
- PySerial 
- Scipy
- PyQT5

## Warning
The COM3 have to be open, the program will try to open it and fail

Keep in mind, this is amateur work so many feature and many response from the motors
have not been considered. 

Keep also in mind that you can break your motors if the use this program without cautions.

Finally most of the Arm class was written for the motor DX-117, the communication 
protocol is the same between all the dynamixel products (this is what they say) but maybe
you will have to adapt some features for your particular motors.

## Documentation
See comment in the code