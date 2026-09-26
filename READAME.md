This is a custom made 2.5d lidar from a 2d lidar (rplidar C1 in this case), I am using a servo to rotate it 180 degrees, I plan to use a stepper motor in the future for better accuracy. 

# NODES
-lidar_driver_node is responsible for communicating to the lidar 
-pan_controller_node is responsible for controlling the movement of servo 
-scan_accumulator_node is responsible for collecting all the vertical scans and stiching them over time to create a point cloud 

# COMMANDS
source install/setup.zsh 
source /opt/ros/humble/setup.zsh
ros2 launch lidar_mapper_ros2 lidar_mapper.launch.py           


# Wired (default — same as just running with no argument)
ros2 launch lidar_mapper_ros2 lidar_mapper.launch.py

# Wireless
ros2 launch lidar_mapper_ros2 lidar_mapper.launch.py mode:=wireless