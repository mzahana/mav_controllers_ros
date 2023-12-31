cmake_minimum_required(VERSION 3.8)
project(mav_controllers_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(mavros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCommand.msg"
  "msg/SE3Command.msg"
  "msg/ControlErrors.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

#
# SE3controller.cpp
#
add_library(se3controller src/SE3Controller.cpp)
ament_target_dependencies(se3controller Eigen3)
# this line to exports the library
ament_export_targets(se3controller HAS_LIBRARY_TARGET)
target_include_directories(se3controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    )


install(
  TARGETS se3controller
  EXPORT se3controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#
# SE3 controller node
#
add_executable(se3controller_node src/se3_controller_node.cpp)
ament_target_dependencies(se3controller_node
                          nav_msgs
                          geometry_msgs
                          mavros_msgs
                          tf2
                          tf2_ros 
                          tf2_geometry_msgs 
                          rclcpp 
                          Eigen3)

target_link_libraries(se3controller_node se3controller)
target_include_directories(se3controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(se3controller_node ${cpp_typesupport_target})


install(TARGETS
  se3controller_node
  DESTINATION lib/${PROJECT_NAME}
)


#
# GeometricAttitudeControl.cpp
#
add_library(geometric_controller src/GeometricAttitudeControl.cpp)
ament_target_dependencies(geometric_controller Eigen3)
# this line to exports the library
ament_export_targets(geometric_controller HAS_LIBRARY_TARGET)
target_include_directories(geometric_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    )


install(
  TARGETS geometric_controller
  EXPORT geometric_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#
# Geometric Attitude controller node
#
add_executable(geometric_controller_node src/geometric_attitude_control_node.cpp)
ament_target_dependencies(geometric_controller_node
                          nav_msgs
                          geometry_msgs
                          mavros_msgs
                          trajectory_msgs
                          tf2
                          tf2_ros 
                          tf2_geometry_msgs 
                          rclcpp 
                          Eigen3)

target_link_libraries(geometric_controller_node geometric_controller)
target_include_directories(geometric_controller_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target_geo_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(geometric_controller_node ${cpp_typesupport_target_geo_node})


install(TARGETS
geometric_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

#
# SE3 controller MAVROS node
#
add_executable(se3controller_mavros_node src/se3controller_mavros_node.cpp)
ament_target_dependencies(se3controller_mavros_node
                          nav_msgs
                          geometry_msgs
                          mavros_msgs
                          tf2
                          tf2_ros 
                          tf2_geometry_msgs 
                          rclcpp 
                          Eigen3)

# target_link_libraries(se3_controller_node se3controller)
target_include_directories(se3controller_mavros_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target_mavros ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(se3controller_mavros_node ${cpp_typesupport_target_mavros})


install(TARGETS
  se3controller_mavros_node
  DESTINATION lib/${PROJECT_NAME}
)

#
# Geometric controller MAVROS node
#
add_executable(geometric_mavros_node src/geometric_mavros_node.cpp)
ament_target_dependencies(geometric_mavros_node
                          nav_msgs
                          geometry_msgs
                          mavros_msgs
                          tf2
                          tf2_ros 
                          tf2_geometry_msgs
                          rclcpp
                          message_filters
                          Eigen3)

# target_link_libraries(se3_controller_node se3controller)
target_include_directories(geometric_mavros_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target_geo_mavros ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(geometric_mavros_node ${cpp_typesupport_target_geo_mavros})


install(TARGETS
  geometric_mavros_node
  DESTINATION lib/${PROJECT_NAME}
)

#
# fixed setpoint test node
#
add_executable(static_setpoint_test_node test/static_setpoint_test.cpp)
ament_target_dependencies(static_setpoint_test_node rclcpp trajectory_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(static_setpoint_test_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target_static_setpoint_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(static_setpoint_test_node ${cpp_typesupport_target_static_setpoint_test})
install(TARGETS
  static_setpoint_test_node
  DESTINATION lib/${PROJECT_NAME}
)

#
# circular trajectory test node
#
add_executable(circular_trajectory_node test/circular_trajectory_node.cpp)
ament_target_dependencies(circular_trajectory_node mavros_msgs nav_msgs tf2 tf2_ros rclcpp)
target_include_directories(circular_trajectory_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target_circular_traj_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(circular_trajectory_node ${cpp_typesupport_target_circular_traj_test})
install(TARGETS
  circular_trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

###########################
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

##############################3

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
