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
chmod +x robot_core_odom.py 
code .  # open VScode >> robot_core_odom.py and copy and pressls
cd ..
code .  # open VScode >> setup.py >> "etry_point =" insert "robot_core_odom = navrobot.robot_core_odom:main"
cd .. x2
colcon build
source install/local_setup.bash
ros2 pkg list    # can see "navrobot"
```
## connect with robot
```
# run micro_ros_agent micro_ros_agent 
ros2 run navrobot robot_core_odom
ros2 topic list

sudo apt install ros-galactic-teleop-twist-keyboard  # install key board control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 topic echo /right_tick   # test topic encoder right
```

### Odometry
```
cd navrobot/src/navrobot/navrobot/
code .   # robot_core_odom.py
cd ..   # x3 navrobot/
colcon build
ros2 run navrobot robot_core_odom

rviz2    # run woke space
```
### odom_nav.py >> ~/navrobot_ws/src/navrobot/navrobot$
```
cd ~/navrobot_ws/src/navrobot/navrobot
touch odom_nave.py  # open copy from google drive
ros2 interface list
ros2 interface show geometry_msgs/msg/Point


```
