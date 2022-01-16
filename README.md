# Turtlebot2-QR-Navigation
We have made a state machine where tb2 reads a qr code and then navigates to a point in a map.

How to launch it

roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/path_to/map.world

roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/path_to/map.yaml

roslaunch turtlebot_rviz_launchers view_navigation.launch

put a QR on the package at right side of map,
then

roslaunch camara turtlebot_qr_paquetes.launch

if you want to change robot path, put another qr while turtlebot is doing any path
