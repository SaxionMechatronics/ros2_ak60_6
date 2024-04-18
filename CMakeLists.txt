cmake_minimum_required(VERSION 3.8)
project(ros2_ak60_6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(ak60_6 src/ak60_6.cpp)
ament_target_dependencies(ak60_6 rclcpp std_msgs sensor_msgs can_msgs std_srvs)
ament_export_dependencies(rosidl_default_runtime)

target_include_directories(ak60_6 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ak60_6 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorReading.msg"
  "srv/SetKpKd.srv"
  "srv/SetCanId.srv"
 )

install(TARGETS ak60_6
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(ak60_6
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
