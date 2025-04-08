# v1.3
* Added a Start and End character for the JSON packet.

# v1.2
* Disabled inside menuconfig, the UART port used for log output.

# v1.1
* Corrected the GPIO config for left and right motors.
* Adjusted the PID constants, and the dT parameter for motor control.
* Added parse for torque and position commands of base motors in the JSON packet receiver.
* Added PWM duty cycle ans torque feedback, and encoder instantaneous PPR tick count as position feedback of the base motors in JSON packet.
* Corrected the encoder directionality.

# v1.0
* Rebuilt the microROS project "robot_base" v1.8 from scratch to exclude the micrROS interface.
* This node receives the velocity for each motor, and sends back the instantaneous velocity of each motor as a feedback.
* The data is exchanged using JSON packets.
