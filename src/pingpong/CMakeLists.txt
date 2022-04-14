cmake_minimum_required(VERSION 3.8)
project(pingpong)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(pingpong_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ping.msg"
  "msg/Pong.msg"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(ping_obsolete src/ping_obsolete.cpp)
ament_target_dependencies(ping_obsolete rclcpp)
rosidl_target_interfaces(ping_obsolete ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(pong_obsolete src/pong_obsolete.cpp)
ament_target_dependencies(pong_obsolete rclcpp)
rosidl_target_interfaces(pong_obsolete ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ping_timer src/ping_timer.cpp)
ament_target_dependencies(ping_timer rclcpp)
rosidl_target_interfaces(ping_timer ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(pong_timer src/pong_timer.cpp)
ament_target_dependencies(pong_timer rclcpp)
rosidl_target_interfaces(pong_timer ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ping_classify src/ping_classify.cpp)
ament_target_dependencies(ping_classify rclcpp)
rosidl_target_interfaces(ping_classify ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(pong_classify src/pong_classify.cpp)
ament_target_dependencies(pong_classify rclcpp)
rosidl_target_interfaces(pong_classify ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  TARGETS
    ping_obsolete pong_obsolete
    ping_timer pong_timer
    ping_classify pong_classify
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
