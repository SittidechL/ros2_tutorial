## Teensy4.0 (robot_firmware.ino)
```
copy file from github "robot_firmware" to arduino and upload to teensy4.0
```

## ros2_tutorial (robot_core_odom.py) >> ~/navrobot_ws/src/navrobot/navrobot$
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
cd microros_ws && source install/local_setup.bash

cd microros_ws
source /opt/ros/galactic/setup.bash
source install/local_setup.bash
ros2 pkg list  # micro_ros_agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 topic list # check topic
ros2 topic pub --once /wheel_command_left std_msgs/msg/Float32 data:\ 20.0\ 
ros2 topic pub --once /wheel_command_right std_msgs/msg/Float32 data:\ 20.0\ 

ros2 topic echo /left_tick
ros2 topic echo /right_tick 

# run micro_ros_agent micro_ros_agent 

cd navrobot_ws && source install/local_setup.bash
ros2 pkg list   # navrobot
ros2 run navrobot robot_core_odom
ros2 topic list  # /cmd_vel /left_tick

sudo apt install ros-galactic-teleop-twist-keyboard  # install key board control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 topic echo /right_tick   # test topic encoder right
```

### Odometry
```
terminal#1
cd navrobot_ws && source install/local_setup.bash
ros2 run navrobot robot_core_odom

terminal#2
source install/local_setup.bash
rviz2

------------
cd navrobot_ws
source /opt/ros/galactic/setup.bash
colcon build
cd src/navrobot/navrobot/
code .   # robot_core_odom.py
cd ..   # x3 navrobot/
colcon build
ros2 run navrobot robot_core_odom

rviz2    # run woke space
```
### odom_nav.py >> ~/navrobot_ws/src/navrobot/navrobot$
```
cd ~/navrobot_ws/src/navrobot/navrobot
touch odom_nav.py  # open copy from google drive
ros2 interface list
ros2 interface show geometry_msgs/msg/Point
python3 odom_nav.py
```
### RPLidar
```
rplidar ros2 github  # from Google
https://github.com/Slamtec/rplidar_ros/tree/ros2
cd navrobot_ws/src
git clode -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ..
colcon bulid
ros2 pkg list # rplidar_ros2
ros2 launch rplidar_ros2 view_rplidar_launch.py 

2:00:00
```
### launch file
```
cd ~/navrobot_ws/src/navrobot/
mkdir launch && cd launch
touch robot_bringup.launch.py  
code .   # copy file from Ros2_nav/launch/robot_bringup.launch.py
cd .. x3
colcon build
ros2 launch navrobot robot_bringup.launch.py   # connect microcontroller
```

### cartographer SLAM
```
cd navrobot_ws/
sudo apt install ros-galactic-cartographer-ros
cd src/navrobot/
mkdir config  # /home/sittidechl/navrobot_ws/src/navrobot/config/cartographer.lua
cd launch     # /home/sittidechl/navrobot_ws/src/navrobot/launch/cartographer.launch.py
colcon build
ros2 launch navrobot robot_bringup.launch.py
ros2 launch navrobot cartographer.launch.py
rviz2
```
### map server
```
cd navrobot
sudo apt install ros-galactic-nav2-map-server
ros2 run nav2_map_server map_saver_cli -f new_map
```


### run
terminal#1
cd navrobot_ws
source /opt/ros/galactic/setup.bash
colcon build
source install/local_setup.bash
ros2 run navrobot robot_core_odom

terminal#2
cd navrobot_ws
source /opt/ros/galactic/setup.bash
cd ~/navrobot_ws/src/navrobot/navrobot
python3 odom_nav.py


terminal#3
cd navrobot_ws
source /opt/ros/galactic/setup.bash
rviz2

terminal#4
cd nav2sim_ws
source /opt/ros/galactic/setup.bash
colcon build
source install/local_setup.bash
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}:~/nav2sim_ws/src/lapras_sim # dir to "nav2sim_ws" path: nav2sim_ws/src/lapras_sim/config/env_config --> double click
ros2 launch lapras_sim lapras_world.launch.py
