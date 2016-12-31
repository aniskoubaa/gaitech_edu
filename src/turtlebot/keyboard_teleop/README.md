# keyboard teleop 

## what is it?

this publisher source code creates a ros node called "key_teleop". a node is just an executable file within a ros package (in this case turtlebot). every node communicates with ros master node, which can be found at ROS_MASTER_URI=http://localhost:11311/.

by reading user inputs, key_teleop publishes (a) linear data and (b) angular data to a topic called "cmd_vel" [1]. 

[1] keyboard_teleop.launch script maps the topic "cmd_vel" to "cmd_vel_mux/input/teleop". this remapping is required since the turtlebot base subscribes to "cmd_vel_mux/input/teleop."