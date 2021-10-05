# repo
https://github.com/ros2/examples/tree/foxy/rclcpp/services

# check for missing dependencies before building
rosdep install -i --from-path xxx --rosdistro foxy -y

1. C++
# build c++
colcon build --packages-select examples_rclcpp_minimal_service
colcon build --packages-select examples_rclcpp_minimal_client

# run c++
ros2 run examples_rclcpp_minimal_service service_main
ros2 run examples_rclcpp_minimal_client  client_main

# code demo c++
```c++
create_client<srv>("service")
create_service<srv>("service",service_callback)
// callback requrst--> response
service_callback(request_header,request,response)
```

2. PYTHON
# build py
colcon build --packages-select examples_rclpy_minimal_service
colcon build --packages-select examples_rclpy_minimal_client

# run py
ros2 run examples_rclpy_minimal_service service
ros2 run examples_rclpy_minimal_client client

**Hints:**
- python is dynamic language, we do not need to re-build after modify code,
  it is easy to test different client request input.

# refenrence
- https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html
- https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
- https://blog.csdn.net/qq_16893195/article/details/113571858