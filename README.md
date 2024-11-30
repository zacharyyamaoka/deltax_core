
This repository provides all of the mid-level software needed to run the Delta XS robot arm with ROS2.

The stack is set up as a ROS metapackage, which organizes our codebase into the following packages:

- **deltax_descriptions**
  - URDF, meshes, etc.
- **deltax_driver**
  - Lower-level code for communicating with hardware (ie motor drivers) and providing a ROS interface to them

## Installation

- Tested on Ubuntu 22.04 LTS and ROS Iron

- Go to src of your workspace:
  ```bash
  cd ~/<your_ws>/src 
  ```
- Clone the code:
  ```bash
  git clone https://github.com/zacharyyamaoka/deltax_core
  ```
- Install dependencies:
  ```bash
  cd ~/<your_ws>
  rosdep install --from-paths src --ignore-src -r -y
  ```
- Build (symlink-install is optional):
  ```bash
  colcon build --symlink-install
  ```
- Source:
  ```bash
  source ~/<your_ws>/install/setup.bash
  ```

## Use Cases

Assumes you are in a sourced workspace
#### View Arm
  ```bash
    ros2 launch deltax_descriptions display.launch.py
  ```


## References

https://docs.deltaxrobot.com/reference/gcodes/useful_commands/ for useful commands

https://docs.deltaxrobot.com/reference/gcodes/gc_xs_v5/ for gcode reference


Code based on:

https://github.com/VanThanBK/deltax_ros_public
https://github.com/VanThanBK/python-deltax/tree/master
https://github.com/deltaxrobot/deltax_pick_place_script


From: https://github.com/VanThanBK/Delta-X-Firmware/blob/master/Delta_Firmware/Geometry.h
