# Install ESP-IDF
* Instructions: https://docs.espressif.com/projects/esp-idf/en/release-v5.2/esp32/get-started/linux-macos-setup.html

# Clone the ESP-IDF component of micro-ROS
* Repository link: https://github.com/micro-ROS/micro_ros_espidf_component/tree/humble
* Branch: humble
* Commit: c5be1627dfa1d05a9a84ce9aed9e97e09790ae46

# Start the micro-ROS node
* ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB1 -b 460800