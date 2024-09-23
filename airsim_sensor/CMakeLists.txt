cmake_minimum_required(VERSION 3.8)
project(airsim_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)    # Ensure this is included
find_package(cv_bridge REQUIRED)      # Ensure this is included if you are using cv_bridge

# Add executable and link dependencies
add_executable(shared_image_reader src/shared_image_reader.cpp)
ament_target_dependencies(shared_image_reader
  rclcpp
  sensor_msgs     # Link sensor_msgs
  cv_bridge       # Link cv_bridge if you are using it
)

# Install Cpp executables
install(TARGETS
  shared_image_reader
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/airsim_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
