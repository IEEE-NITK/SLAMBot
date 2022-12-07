# SLAMBot

Clone this repository in your ROS workspace and build your packages using `catkin build`.

Launch the TurtleBot3 simulation using the following command:
```bash
roslaunch slambot_simulation turtlebot_simulation.launch
```

Basic move code C++ template is in [this](/slambot_simulation/src/turtlebot3_move.cpp) file. C++ code needs to be compiled in order to run. You can compile by adding your C++ file to CMakeLists.txt like [this](/slambot_simulation/CMakeLists.txt?plain=1#L141-L143), and then running `catkin build`.


Arduino code for uploading to your embedded board is [here](/slambot_arduino/differential_drive/differential_drive.ino). Compile and upload the code using Arduino IDE. Run the rosserial server node on your connected computer to connect the ROS node on the board. This node needs to be running to "*activate*" your ROS node on your arduino board.

```bash
rosrun rosserial_arduino serial_node.py
```