cmake_minimum_required(VERSION 3.5)
project(imu_transformer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(library_name ${PROJECT_NAME})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  message_filters
  rclcpp 
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  Eigen3
)

find_package(ament_cmake REQUIRED COMPONENTS
    ${dependencies}
)
add_library(${library_name} INTERFACE 
)

include_directories(${library_name}
    include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${library_name} INTERFACE 
    ${dependencies}
)
#ament_target_dependencies(${library_name}
#    ${dependencies}
#)

ament_export_targets(${library_name})

ament_export_include_directories(include 
    ${EIGEN3_INCLUDE_DIRS}
    )
ament_export_dependencies(${dependencies})

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION include 
)
install(
    TARGETS ${library_name}
    EXPORT ${library_name}
    INCLUDES DESTINATION include
)

ament_package()
