#!/bin/bash -e

sudo chmod 777 /dev/ttyUSB0
sleep 2
# sudo chmod 777 /dev/ttyACM0
sleep 2
roslaunch  agv_tcp_velocity laser.launch&
sleep 1
roslaunch ira_laser_tools laserscan_multi_merger.launch&
sleep 1
# rosrun map_server map_server /home/sjtuwhl/ROBOTLAB_WS/src/agv_tcp_velocity/map/map.yaml&
sleep 1
# roslaunch agv_tcp_velocity pure_matcher.launch&
roslaunch agv_tcp_velocity my_gmapping_launch.launch
sleep 1
# roslaunch amcl amcl_omni.launch&
sleep 1
# roslaunch agv_tcp_velocity move_base.launch&
sleep 5
rosrun soccer_maxon soccer_maxon_node&
sleep 2
# rosrun arduino_io arduino_io&
sleep 2
# roslaunch move_forward move_and_avoid_dynamic_object.launch&

sleep 3
