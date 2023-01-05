# SLAMBot

[![GitHub issues](https://img.shields.io/github/issues/IEEE-NITK/SLAMBot?color=red&label=Issues&style=flat)](https://github.com/IEEE-NITK/SLAMBot/issues)
[![GitHub license](https://img.shields.io/github/license/IEEE-NITK/SLAMBot?color=green&label=License&style=flat)](https://github.com/IEEE-NITK/SLAMBot/blob/main/LICENSE)
![Ubuntu](https://img.shields.io/badge/Ubuntu%2020.04-%E2%9C%94-blue)
![ROS Noetic](https://img.shields.io/badge/ROS%20Noetic-%E2%9C%94-blue)

### Table of Contents
<ol>
    <li>
        <a href="#introduction">Introduction</a>
        <ul>
            <li><a href="#technologies-used">Technologies Used</a></li>
        </ul>
    </li>
    <li>
        <a href="#getting-started">Getting Started</a>
        <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
        </ul>
    </li>
</ol>

<hr>

## Introduction

<b>SLAM</b> (<b>S</b>imultaneous <b>L</b>ocalization <b>A</b>nd <b>M</b>apping) is an essential technology used in robotics that helps robots to estimate their position and orientation on a map while creating the map of the environment to carry out autonomous activities. 

![Alt text](Assets/slam.png)
*Turtlebot using SLAM to navigate across a map*

This project aims to put together a mobile robot similar to a TurtleBot. A TurtleBot is a low-cost, personal robot kit with open source software.

![Alt text](Assets/slambot.png)
*SLAMBot*

### Technologies Used
[![Tech_Used](https://skills.thijs.gg/icons?i=ros,py,cpp,arduino,raspberrypi&theme=dark)](https://skills.thijs.gg)

## Getting Started

### Prerequisites

* Ubuntu 20.04
* ROS Noetic

### Installation

Clone this repository in a ROS workspace and build the packages using `catkin build`.

Launch the TurtleBot3 simulation using the following command:
```bash
roslaunch slambot_simulation turtlebot_simulation.launch
```

Basic move code C++ template is in [this](/slambot_simulation/src/turtlebot3_move.cpp) file. C++ code needs to be compiled in order to run. You can compile by adding your C++ file to CMakeLists.txt like [this](/slambot_simulation/CMakeLists.txt?plain=1#L141-L143), and then running `catkin build`.


Arduino code for uploading to your embedded board is [here](/slambot_arduino/differential_drive/differential_drive.ino). Compile and upload the code using Arduino IDE. Run the rosserial server node on your connected computer to connect the ROS node on the board. This node needs to be running to "*activate*" your ROS node on your arduino board.

```bash
rosrun rosserial_arduino serial_node.py
```