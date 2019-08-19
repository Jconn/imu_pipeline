cmake_minimum_required(VERSION 3.5)
project(imu_transformer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(message_filters REQUIRED)


include_directories(
    include
    ${ament_INCLUDE_DIRS}
    )
set(executable_name imu_transformer)

add_executable(
    ${executable_name}
    src/imu_transformer_node.cpp
    )

set(dependencies
  rclcpp
  rclcpp_lifecycle
  message_filters
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  tf2
  nav2_util
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)




ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

install(DIRECTORY include/
    DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


install(TARGETS
    ${executable_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
    )

ament_export_libraries(
    ${executable_name})

ament_package()
