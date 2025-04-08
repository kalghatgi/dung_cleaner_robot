# v1.2
* Corrected the odometry calculation formula by excluding the time delta (dT).
* Corrected the sign of the obtained velocity before odometry calculation.
* The packet reader now expects a Start and End character to ensure proper parsing of data.

# v1.1
* Changed the data reading method.
* Added a parameter to accept data from specified serial port.
* Corrected the JSON parse syntax.

# v1.0
* Project reference from robot_base, a ROS node, to work directly with the devices connected to the USB ports.
