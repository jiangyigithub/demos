# build
colcon build --packages-select examples_rclcpp_minimal_service
colcon build --packages-select examples_rclcpp_minimal_client

# run
ros2 run examples_rclcpp_minimal_service service_main
ros2 run examples_rclcpp_minimal_client  client_main