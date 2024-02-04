# Turtle Carnage Game

Welcome to the **Turtle Carnage** game repository! 🐢🚀

## Software Requirements
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10

## Environment Setup
1. Create a ROS2 workspace:
   ```bash
   mkdir ros2_ws
   cd ros2_ws
   mkdir src
   colcon build

2. If colcon is not installed, run:
   ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
3. Add the ROS2 workspace to your ~/.bashrc:
   ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc

## Organize Your Folders
Place the following folders inside the `src` folder:
- `turtle_carnage`
- `my_robot_interfaces`
- `my_robot_bringup`


## Build Your Workspace
```bash
  cd ros2_ws
  colcon build
  source ~/.bashrc
```
## Launch Your Turtle Carnage App
```bash
cd
ros2 launch my_robot_bringup turtle_carnage_app.launch.py
```

Feel free to explore the code and have fun with the turtles! 🐢🎮
