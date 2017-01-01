# keyboard teleop 

## what is it?

this publisher source code creates a ros node called "key_teleop". a node is just an executable file within a ros package (in this case turtlebot). every node communicates with ros master node, which can be found at ROS_MASTER_URI=http://localhost:11311/.

by reading user inputs, key_teleop publishes (a) linear data and (b) angular data to a topic called "cmd_vel." note that keyboard_teleop.launch script maps the topic "cmd_vel" to "cmd_vel_mux/input/teleop". this remapping is required since the turtlebot base (the "mobile_base_nodelet_manager" node) subscribes to "cmd_vel_mux/input/teleop."

## interrogating nodes

### what are active nodes?
```
rosnode list
```

### can i see topics/services for a particular node?
```
rosnode info /key_teleop
```
we see the following:
* key_teleop publishes to "/cmd_vel_mux/input/teleop" and "/rosout [rosgraph_msgs/Log]"
* key_teleop doesnt have any subscriptions.
* key_teleop provides the following services: "/turtlebot_teleop_keyboard/set_logger_level" and "/turtlebot_teleop_keyboard/get_loggers"

additionally, we see that the topic "/mobile_base_nodelet_manager" is subscribed by another node called "/mobile_base_nodelet_manager". you can verify this by doing:
```
rosnode info /mobile_base_nodelet_manager
```

### can i see a list of all topics?
```
rostopic list
```

### can i inspect a specific topic?
obviously rostopic follows the same pattern as rosnode.
```
rostopic info /cmd_vel_mux/input/teleop
```
youll see that the topic is strongly typed (Type: geometry_msgs/Twist). publisher is /turtlebot_teleop_keyboard (http://localhost:47731/) and subscriber is /mobile_base_nodelet_manager (http://localhost:47584/)