# ros2_tutorial
```
source /opt/ros/galactic/setup.bash
source /opt/ros/galactic/setup.bash
mkdir navrobot_ws && cd navrobot_ws     # Keep code
mkdir src && cd src
ros2 pkg create --build-type ament_python naverobot # create package navrobot
cd navrobot_ws/
colcon build   # sudo apt install python3-colcon-commond-extensions
colcon build --packages-select navrobot
cd src/navrobot/navrobot
touch robot_core_odom.py
chmod +x robot 

```
