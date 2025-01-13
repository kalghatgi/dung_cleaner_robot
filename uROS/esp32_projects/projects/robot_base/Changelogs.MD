# v1.6
* Converted the imu data from NWU to ENU frame as required by the robot_localisation_node (ekf). [Reference: https://answers.ros.org/question/378777/#:~:text=which%20specifies%20the-,ENU%20frame,-.%20That%20means%20you]

# v1.5
* Corrected the function that sets the duty cycle to each motor.

# v1.4
* Improved the function that checks the wheel_velocity timeout.

# v1.3
* Modified the GPIOs for ease of connections while making the zero-PCB.

# v1.2
* Modified the PWM and Digital_output pin config for the selected motor driver.
* Modified the speed control function for controlling the speed and direction of both motors using the selected driver.
* Increased the microros uart baud rate to 460800.
* Removed unused components, viz. the ADC and CAN.

# v1.1
* Modified the project to work with esp-idf v5.2.3 and microros humble.

# v1.0
* Copied the project from repo: Autonomous Mobile Robot (private to Hsaka_Igtahglak)
