### robotic_pusher
ROS package for interacting with objects, learning their weight and pushing them to a desired distance

Done in simulation with Gazebo and TIAGo robot


# Before launch:
add

    <arg name="world_name" value="$(find robotic_pusher)/worlds/$(arg world).world"/>
    
into `simulation_ws/src/pal_gazebo_worlds/launch/pal_gazebo.launch`

Build the robotic pusher package.
If desired: Change parameters in config/params.yaml file (desired distance, specific cubes to be spawned, hardcoded pushing velocities)


# Launch instructions:

First in one terminal:

    roslaunch robotic_pusher test.launch
    # OR launch not in the center (not working yet, weird bug):
    roslaunch robotic_pusher test.launch gzpose:="-x -1.0 -y -1.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0"

In another terminal:

    cd catkin_ws/src/robotic_pusher (this is where the current example ont.txt, with the queries from training, is located)
    
    # Training
    rosrun robotic_pusher get_action_node -t1
    # Execution
    rosrun robotic_pusher get_action_node

    If prolog takes too long to start -> stop, try again.
