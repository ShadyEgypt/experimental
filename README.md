# Experimental Robotics Laboratory – Assignment 1

This repository contains the solution for **Assignment 1** of the
**Experimental Robotics Laboratory** course.

## Assignment Description

The goal of the assignment is to:

- Spawn a robot in simulation in an environment containing **5 ArUco markers**
- Detect all marker IDs using the robot camera
- Move the robot sequentially toward each marker in **ascending ID order**
- Center the marker in the camera image using **visual servoing**
- Publish a processed image with the detected marker highlighted

## Demo Video

A short video demonstrating the robot behavior, ArUco marker detection,
visual servoing, and sequential navigation is available on YouTube.

[![Assignment 1 Demo Video](https://img.youtube.com/vi/N0DpCXptaio/0.jpg)](https://www.youtube.com/watch?v=N0DpCXptaio)

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
git clone --recurse-submodules https://github.com/ShadyEgypt/experimental.git
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
ros2 launch ex_assignment1 main.launch.py
```
Terminal 2: inside ros2_ws/
```
source install/setup.bash
ros2 run ex_assignment1 observer
```

## Author
Shady Abdelmalek