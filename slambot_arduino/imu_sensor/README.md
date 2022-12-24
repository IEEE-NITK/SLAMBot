# Interfacing a MPU 9255 Sensor with Arduino Nano/Uno and ROS

Follow [this](https://github.com/Bill2462/MPU9255-Arduino-Library) GitHub repository to connect a MPU9255 IMU with a Arduino Board (Nano/Uno/Mega). Tested with Uno and Nano.

Follow the pin connections as given in the README. Import the library and use the given paramters as in the sample code.

Interface with ROS by declaring Publishers of message type **geometry_msgs/Vector3** to publish the IMU measurements to ROS after running `serial_node.py` (refer to main README).