cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(mtc_tutorial src/mtc_node.cpp)
ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
# target_include_directories(mtc_tutorial PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)

add_executable(minimal_mtc_tutorial src/minimal.cpp)
ament_target_dependencies(minimal_mtc_tutorial moveit_task_constructor_core rclcpp)
# target_include_directories(minimal_mtc_tutorial PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
target_compile_features(minimal_mtc_tutorial PUBLIC c_std_99 cxx_std_17)

install(TARGETS mtc_tutorial minimal_mtc_tutorial
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/motion_planning_python_api_tutorial.py
  scripts/motion_planning_python_api_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
