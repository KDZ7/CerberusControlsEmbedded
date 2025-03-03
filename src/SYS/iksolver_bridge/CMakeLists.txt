cmake_minimum_required(VERSION 3.8)
project(iksolver_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(ENABLE_DEBUG "Enable debug log output" ON)
if(ENABLE_DEBUG)
  add_compile_definitions(__DEBUG__)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(log REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgroup REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(iksolver_bridge src/iksolver_bridge.cpp)
ament_target_dependencies(iksolver_bridge log rclcpp msgroup yaml-cpp)

target_link_libraries(iksolver_bridge ${YAML_CPP_LIBRARIES})

install(TARGETS
  iksolver_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
