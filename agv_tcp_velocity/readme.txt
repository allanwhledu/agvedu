roslaunch  agv_tcp_velocity laser.launch
roslaunch ira_laser_tools laserscan_multi_merger.launch
rosrun map_server map_server /home/sjtuwhl/ROBOTLAB_WS/src/agv_tcp_velocity/map/map.yaml
rviz
roslaunch agv_tcp_velocity pure_matcher.launch
roslaunch amcl amcl_omni.launch

sudo chmod 777 /dev/ttyUSB0
电机

sudo chmod 777 /dev/ttyACM0
arduino
rosrun arduino_io arduino_io

rosrun soccer_maxon soccer_maxon_node

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


roslaunch move_forward move_and_avoid_dynamic_object.launch
roslaunch agv_tcp_velocity move_base.launch
