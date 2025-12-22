cmake_minimum_required(VERSION 3.5)
project(dcs_nav_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenCV REQUIRED)
# CasADi C++ (required for optimization). Make optional to avoid blocking builds when unavailable.
# Try to find CasADi - may be installed as casadi or CasADi
find_package(casadi QUIET)
if(NOT casadi_FOUND)
  # Try alternative package name
  find_package(CasADi QUIET)
  if(CasADi_FOUND)
    set(casadi_FOUND TRUE)
  endif()
endif()

set(DCS_NAV_PLUGIN_SRCS
  src/hybrid_a_star.cpp
)
if(casadi_FOUND)
  list(APPEND DCS_NAV_PLUGIN_SRCS src/hyperplane_planner.cpp)
endif()

add_library(dcs_nav_plugin SHARED ${DCS_NAV_PLUGIN_SRCS})

target_include_directories(dcs_nav_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(dcs_nav_plugin
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
if(casadi_FOUND)
  # Link CasADi library - check for different target names
  if(TARGET casadi)
    target_link_libraries(dcs_nav_plugin casadi)
    message(STATUS "Linking HyperplanePlanner with CasADi (target: casadi)")
  elseif(TARGET casadi::casadi)
    target_link_libraries(dcs_nav_plugin casadi::casadi)
    message(STATUS "Linking HyperplanePlanner with CasADi (target: casadi::casadi)")
  else()
    # Fall back to library variable
    find_library(CASADI_LIBRARY NAMES casadi PATHS /usr/local/lib NO_DEFAULT_PATH)
    if(CASADI_LIBRARY)
      target_link_libraries(dcs_nav_plugin ${CASADI_LIBRARY})
      target_include_directories(dcs_nav_plugin PUBLIC /usr/local/include)
      message(STATUS "Linking HyperplanePlanner with CasADi (library: ${CASADI_LIBRARY})")
    else()
      message(FATAL_ERROR "CasADi found but no valid target or library available")
    endif()
  endif()
  target_compile_definitions(dcs_nav_plugin PUBLIC DCS_WITH_CASADI=1)
else()
  message(WARNING "CasADi not found - HyperplanePlanner will not be built. Install CasADi to enable it.")
endif()

ament_target_dependencies(dcs_nav_plugin
  rclcpp
  nav2_core
  nav2_util
  nav2_msgs
  nav2_costmap_2d
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS dcs_nav_plugin
  EXPORT dcs_nav_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(dcs_nav_plugin)
ament_export_dependencies(
  rclcpp
  nav2_core
  nav2_util
  nav2_msgs
  nav2_costmap_2d
  geometry_msgs
  tf2_ros
  pluginlib
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
