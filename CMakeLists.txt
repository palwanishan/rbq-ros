cmake_minimum_required(VERSION 3.5)
project(rbq_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Qt5 COMPONENTS Core Network REQUIRED)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(rbq_ros_interface 
  main.cpp
    headers/rbq_api.h
    headers/Publisher.h
    headers/Subscriber.h
  )

target_include_directories(
  rbq_ros_interface 
    PUBLIC  
      ${EIGEN3_INCLUDE_DIR}
      ${QT_INCLUDES} 
  )

ament_target_dependencies(
  rbq_ros_interface 
    rclcpp std_msgs sensor_msgs  
    ${EIGEN3_LIBRARIES}
    Qt5::Core Qt5::Network
  )

install(TARGETS
  rbq_ros_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
