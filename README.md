# Bringup
This package is for general bringup of the robot. The general functions of the scripts and launch files are described below. For information on setting up a docker environment for these files on an NVIDIA Jetson, refer to [EnvironmentSetup.md](EnvironmentSetup.md).

## Scripts
- **mcu_comms.py**: This file handles communication between the Jetson (onboard computer) and the microcontroller (which is used for motor control and low level sensors). Communication is handled through the SPI protocol and occurs at 1 MHz baud rate. This script sends wheel velocity commands via SPI, and receives IMU and odometry data from the MCU.
- **get_known_points.py**: This scripts gets known points and stores them to a .txt file. This is to enable transformation between different maps.

## Launch
The primary launch files are `minimal.launch` and `robot.launch` and `get_reference_points.launch`. Other launch files are provided for convenience.
- **minimal.launch**: Starts the minimal scripts needed to run the robot (mcu_comms and the twist multiplexer).
- **get_reference_points.launch**: Localizes and starts the get_known_points script. Used to get the known points needed for the transformation matrix.
- **sense_and_map.launch**: Launches the stack necessary for mapping. (mcu_comms, twist multiplexer, LIDAR, Camera, SLAM)
- **sense_and_localize.launch**: Launches the stack necessary for localizing within a new map. (mcu_comms, twist multiplexer, LIDAR, Camera, localization)
- **short.launch**: Launches the stack for operating the short robot (mcu_comms, twist multiplexer, LiDAR, Camera, localization, object detection, navigation, dds communication)
- **tall.launch**: Launches the stack for operating the tall robot (mcu_comms, twist multiplexer, LiDAR, Camera, localization, object detection, navigation, dds communication)
- **multi_agent_short.launch**: Launches the short.launch stack, but enables multi-agent planning

## Using This Package
To run the launch files, use a call similar to:
```
roslaunch mattbot_bringup <launch file> 
```

**Author**: Matthew Sato, Engineering Informatics Lab, Stanford University

**License**: This package is released under the [MIT license](LICENSE).
