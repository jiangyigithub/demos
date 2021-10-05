# build c++
colcon build --packages-select examples_rclcpp_minimal_service
colcon build --packages-select examples_rclcpp_minimal_client

# run c++
ros2 run examples_rclcpp_minimal_service service_main
ros2 run examples_rclcpp_minimal_client  client_main

# build py
colcon build --packages-select examples_rclpy_minimal_service
colcon build --packages-select examples_rclpy_minimal_client

# run py
ros2 run examples_rclpy_minimal_service service
ros2 run examples_rclpy_minimal_client client

- https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html
- https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
- https://blog.csdn.net/qq_16893195/article/details/113571858