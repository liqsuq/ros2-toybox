cmake_minimum_required(VERSION 3.8)
project(pingpong)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ping.msg"
  "msg/Pong.msg"
)

ament_export_dependencies(
  ament_cmake
  rclcpp
  rosidl_default_runtime
  rclcpp_components
)

add_library(ping_compo SHARED src/ping_compo.cpp)
target_compile_definitions(ping_compo PRIVATE "PINGPONG_DLL")
ament_target_dependencies(ping_compo rclcpp rclcpp_components)
rosidl_target_interfaces(ping_compo ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(ping_compo "pingpong::PingNode")

add_library(pong_compo SHARED src/pong_compo.cpp)
target_compile_definitions(pong_compo PRIVATE "PINGPONG_DLL")
ament_target_dependencies(pong_compo rclcpp rclcpp_components)
rosidl_target_interfaces(pong_compo ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(pong_compo "pingpong::PongNode")

add_library(timer_compo SHARED src/timer_compo.cpp)
target_compile_definitions(timer_compo PRIVATE "PINGPONG_DLL")
ament_target_dependencies(timer_compo rclcpp rclcpp_components)
rosidl_target_interfaces(timer_compo ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(timer_compo "pingpong::TimerNode")

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

add_executable(ping_executor src/ping_executor.cpp)
ament_target_dependencies(ping_executor rclcpp)
rosidl_target_interfaces(ping_executor ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(pong_executor src/pong_executor.cpp)
ament_target_dependencies(pong_executor rclcpp)
rosidl_target_interfaces(pong_executor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ping_compose src/ping_compose.cpp)
target_link_libraries(ping_compose ping_compo)
ament_target_dependencies(ping_compose rclcpp)
add_executable(pong_compose src/pong_compose.cpp)
target_link_libraries(pong_compose pong_compo)
ament_target_dependencies(pong_compose rclcpp)

add_executable(timer_compose src/timer_compose.cpp)
target_link_libraries(timer_compose timer_compo)
ament_target_dependencies(timer_compose rclcpp)

ament_export_include_directories(include)

ament_export_libraries(ping_compo pong_compo timer_compo)

install(
  DIRECTORY include/pingpong
  DESTINATION include
)

install(
  TARGETS
    ping_compo pong_compo timer_compo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ping_obsolete pong_obsolete
    ping_timer pong_timer
    ping_classify pong_classify
    ping_executor pong_executor
    ping_compose pong_compose
    timer_compose
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
