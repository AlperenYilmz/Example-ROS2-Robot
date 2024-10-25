# Example-ROS2-Robot
## Tools
- ROS2 Humble, RViz2 and all other packages installed with:
```console
apt install ros-humble-desktop
```

## Installation
- Create a ament_cmake package in your ROS2 workspace and add these files to that package, build with colcon, then launch with:
```console
ros2 launch "yourpkgname" display_launch.launch.xml
```

## Notes
- Model is not (yet) Gazebo compatible since it lacks the physical information (mass, gravity, etc.)
