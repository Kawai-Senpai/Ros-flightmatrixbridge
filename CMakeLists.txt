cmake_minimum_required(VERSION 3.5)
project(flightmatrix_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m pip install flightmatrixbridge
    OUTPUT_VARIABLE pip_output
    ERROR_VARIABLE pip_error
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()