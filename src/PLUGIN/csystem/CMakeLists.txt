cmake_minimum_required(VERSION 3.8)
project(csystem)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_DEBUG "Enable debug log output" OFF)
if(ENABLE_DEBUG)
  add_compile_definitions(__DEBUG__)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(log REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(controller_interface csystem_plugin.xml)

add_library(csystem src/csystem.cpp)
add_library(csystem::Csystem ALIAS csystem)
target_compile_features(csystem PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(csystem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  csystem
  "log"
  "rclcpp_lifecycle"
  "std_msgs"
  "sensor_msgs"
  "realtime_tools"
  "control_toolbox"
  "controller_interface"
  "hardware_interface"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(csystem PRIVATE "CSYSTEM_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS csystem
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  csystem
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
