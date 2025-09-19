# Pronto ROS 2 Node

## Usage

### SOLO12

To run the Pronto ROS 2 node for the SOLO12 robot, use the following
```shell
ros2 launch pronto_node.launch.py xacro_pkg:=pronto_solo12 xacro_name:=solo12.urdf config_name:=solo12.yaml
```

To run both the Pronto ROS 2 node and a pre-recorded bag, use
```shell
ros2 launch pronto_bag_test.launch.py xacro_pkg:=pronto_solo12 xacro_name:=solo12.urdf config_name:=solo12.yaml
```