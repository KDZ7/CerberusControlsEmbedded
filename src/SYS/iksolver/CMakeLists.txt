cmake_minimum_required(VERSION 3.8)
project(iksolver)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(msggroup REQUIRED)
find_package(msgwaypoint REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

add_executable(iksolver src/iksolver.cpp src/main.cpp)
ament_target_dependencies(iksolver
  log
  rclcpp_lifecycle
  msggroup
  msgwaypoint
  sensor_msgs
  yaml-cpp
  orocos_kdl_vendor
  kdl_parser
  urdf
)

target_include_directories(iksolver PRIVATE include)
target_link_libraries(iksolver ${YAML_CPP_LIBRARIES} ${OROCOS_KDL_LIBRARIES})

install(TARGETS
  iksolver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
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
