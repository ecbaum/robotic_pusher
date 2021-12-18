# robotic_pusher_ROS
ROS package for interacting with objects, learning their weight and pushing them to a desired distance

Done in simulation with Gazebo and TIAGo robot

Launch instructions:

First:
roslaunch robotic_pusher test.launch

rosrun robotic_pusher color_extractor_node.py

Second:
rosrun robotic_pusher get_action_node -t1