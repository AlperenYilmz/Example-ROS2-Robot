# Example-ROS2-Robot

Platform: ROS2 Humble, colcon

## Info
- Gazebo compatible, physical tank object. Meshes used in the model are not shown in Gazebo.
- No drive plugins incorporated.
- Seperate launch files for RViz2 and Gazebo respectively.
- Also a short script can be found as **mesh_bounding_box.py** that calculates the size of the smallest bounding box of a mesh file.

## Dependencies
- ROS2 Humble, RViz2, Gazebo and all other packages installed with:
```console
apt install ros-humble-desktop ros-humble-gazebo* ros-humble-rviz2
```

## Installation
- Create a ament_cmake package named as tank_robot_desc in your ROS2 workspace and add these files to that package, then build as:
```console
colcon build --packages-select tank_robot_desc
```

## Run
- There's 2 different types of launch files for Gazebo in this package. Differs only by libraries imported.
- You can add your own RViz2 config file in **rviz2_cfg** folder and un-comment the line in **rviz2.launch.xml**. Other Rviz2 launch file does not have config parameter.
- Launch the gazebo model as:
```console
ros2 launch tank_robot_desc gazebo.launch.py
```
- And Rviz2 model as:
```console
ros2 launch tank_robot_desc rviz2.launch.xml
```
