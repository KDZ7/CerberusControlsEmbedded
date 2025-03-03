cmake_minimum_required(VERSION 3.8)
project(hwsystem_ag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_DEBUG "Enable debug log output" ON)
if(ENABLE_DEBUG)
  add_compile_definitions(__DEBUG__)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(log REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(idevice REQUIRED)
find_package(group_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(hardware_interface hwsystem_ag_plugin.xml)


add_library(hwsystem_ag src/hwsystem_ag.cpp)
add_library(hwsystem_ag::hwsystem_ag ALIAS hwsystem_ag)
target_link_libraries(hwsystem_ag ${YAML_CPP_LIBRARIES})
target_compile_features(hwsystem_ag PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(hwsystem_ag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  hwsystem_ag
  "rclcpp"
  "hardware_interface"
  "idevice"
  "group_action"
  "yaml-cpp"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hwsystem_ag PRIVATE "HWSYSTEM_AG_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS hwsystem_ag
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
  hwsystem_ag
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
