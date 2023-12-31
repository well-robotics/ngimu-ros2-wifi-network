cmake_minimum_required(VERSION 3.8)
project(ros2-ngimu-wifi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set (CMAKE_CXX_STANDARD 17)
include(GNUInstallDirs)
include_directories(include)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(std_msgs REQUIRED) 
 
add_executable(${PROJECT_NAME} src/ngimu_wifi.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/liboscpack.a)

ament_target_dependencies(${PROJECT_NAME} rclcpp  
std_msgs
sensor_msgs)        

install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

 
  
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_export_dependencies(sensor_msgs std_msgs geometry_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


ament_package()
