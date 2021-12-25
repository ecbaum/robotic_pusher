### robotic_pusher
ROS package for interacting with objects, learning their weight and pushing them to a desired distance

Done in simulation with Gazebo and TIAGo robot


# Before launch:
add

    <arg name="world_name" value="$(find robotic_pusher)/worlds/$(arg world).world"/>
    
into `simulation_ws/src/pal_gazebo_worlds/launch/pal_gazebo.launch` 


# Launch instructions:

First:

    roslaunch robotic_pusher test.launch

Second:

    rosrun robotic_pusher get_action_node -t1
