cmake_minimum_required(VERSION 3.8)
project(hidraw)

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
find_package(iio REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(iio hidraw_plugin.xml)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

add_library(hidraw src/hidraw.cpp)
add_library(hidraw::Hidraw ALIAS hidraw)
target_compile_features(hidraw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(hidraw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  hidraw
  "iio"
  "pluginlib"
)
target_link_libraries(hidraw
  ${HIDAPI_LIBRARIES}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hidraw PRIVATE "HIDRAW_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS hidraw
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
  hidraw
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
