cmake_minimum_required(VERSION 3.5)
project(lidar_parking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(aboat_interfaces REQUIRED)
find_package(clustering_interfaces REQUIRED)


include_directories(include
#${PCL_INCLUDE_DIRS}
#${rclcpp_INCLUDE_DIRS}
#${aboat_interfaces_INCLUDE_DIR}
${clustering_interfaces_INCLUDE_DIR}
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_parking src/main.cpp src/LidarParkingNode.cpp include/LidarParkingNode.h)

ament_target_dependencies(lidar_parking
  rclcpp
  sensor_msgs
  pcl_conversions
  std_msgs
  #aboat_interfaces
  clustering_interfaces
)

target_link_libraries (lidar_parking ${PCL_LIBRARIES})

install(TARGETS lidar_parking DESTINATION lib/${PROJECT_NAME})

ament_package()
