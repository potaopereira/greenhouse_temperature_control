cmake_minimum_required(VERSION 3.8)
project(temperature_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(greenhouse_msg_srv REQUIRED)

add_executable(
    temperature_sensor
    src/main.cpp
    src/temperature_sensor.cpp
)
ament_target_dependencies(
    temperature_sensor
    rclcpp
    greenhouse_msg_srv
)
target_include_directories(
    temperature_sensor
    PUBLIC
    include
)

# install(TARGETS) section so that ros2 run can find executable
install(TARGETS
    temperature_sensor
    DESTINATION lib/${PROJECT_NAME}
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
