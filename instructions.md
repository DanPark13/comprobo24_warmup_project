# How to Set up the Robot

## To Run the Robot

1. `ping <ip of robot>`

2. `ros2 launch neato_node2 bringup.py host:=<ip of robot>`

## To Build

Source: `source install/local_setup.bash`

Build: `colcon build`

Run: `ros2 run warmup_project hello_world`