# 3806ICT_Assignment3
Samuel Keller s5093941
James Rutherford s5132147

To run the simulation, the following steps are required:
1.	Navigate to the catkin_ws directory with the command ‘cd ~/catkin_ws’
2.	Compile ROS packages using the command ‘catkin_make’
3.	Source the environment with the command ‘source devel/setup.bash’
4.		Launch Gazebo simulated environment with ‘roslaunch grid_world launch_world.launch’. This will launch the gazebo world, and initialise the sen-sor_node and sensor_processing_node.
5.		In a new terminal window, start the ai_interaction_node using the command ‘rosrun grid_world ai_interaction_node.py’
6.		In a new terminal window, start the execution_node using the command ‘rosrun grid_world ‘execution_node.py’
