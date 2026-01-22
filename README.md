# Experimental Robotics Laboratory – Assignment 2

This repository contains the solution for **Assignment 2** of the
**Experimental Robotics Laboratory** course.

## Assignment Description

The goal of the assignment is the same as assignment 1 in addition to the following:

- use plansys2 pkg to automate different actions and states of the project.
- use nav2 pkg to help the robot localize itself and navigate given that the environment is bigger and the aruco markers are not visible to the robot where it is spawned.

## Demo Video

A short video demonstrating the robot behavior is available on YouTube.

[![Assignment 1 Demo Video](https://img.youtube.com/vi/sXCSyjq2Zyc/0.jpg)](https://youtu.be/sXCSyjq2Zyc?si=wqBbBku3NDE3dnwQ)

## Documentation

The complete technical documentation (architecture, nodes, topics, control
logic, and instructions) is generated using **Sphinx** and hosted as a static
website.

👉 **Sphinx HTML Documentation:**  
http://experimental-robotics-assignment1-docs.s3-website.eu-north-1.amazonaws.com/


## Clone the Repository (with Submodules)

This repository uses Git submodules (e.g. the ROS 2 workspace inside
`ros2_ws`).  
To clone everything correctly, use:

```bash
git clone -b assignment2 https://github.com/ShadyEgypt/experimental.git
cd experimental/ros2_ws/src
git clone -b main https://github.com/ShadyEgypt/ex_assignment2.git
git clone -b ex_assignment2 https://github.com/ShadyEgypt/erl1_sensors.git
git clone -b ex_assignment2 https://github.com/ShadyEgypt/aruco_marker_gazebo.git
git clone -b assignment2 https://github.com/ShadyEgypt/ex_assignment1_interfaces.git
git clone -b main https://github.com/ShadyEgypt/ex_assignment1.git
git clone -b ex_assignment2 https://github.com/ShadyEgypt/erl1.git
git clone -b shady-mods https://github.com/ShadyEgypt/ros_aruco_opencv.git
git clone -b main https://github.com/ShadyEgypt/plansys_interface.git
git clone -b main https://github.com/ShadyEgypt/ros2_navigation.git
git clone -b main https://github.com/CarmineD8/my_opencv.git
```
To build the ws
```
cd ros2_ws
sudo rm -rf build log install
chmod +x fix_ros2_ws_paths.sh
./fix_ros2_ws_paths.sh
colcon build
```
Before you run the solution, check the world file inside erl pkg and remove any uncessary models that may not be on your machine.

Terminal 1: inside ros2_ws/
```
source install/setup.bash
ros2 launch ex_assignment2 main.launch.py
```

Terminal 2: inside ros2_ws/
```
source install/setup.bash
ros2 launch ros2_navigation localization.launch.py
```

Terminal 3: inside ros2_ws/
```
source install/setup.bash
ros2 launch ros2_navigation navigation.launch.py
```

Terminal 4: inside ros2_ws/
```
source install/setup.bash
ros2 launch plansys_interface distributed_actions.launch.py
```

Terminal 5: inside ros2_ws/
```
source install/setup.bash
ros2 run plansys2_terminal plansys2_terminal
```
Terminal 5: make sure the problem has been parsed
```
get problem goal
```
Terminal 5: get the plan and run it
```
get plan
run
```
## Author
Shady Abdelmalek