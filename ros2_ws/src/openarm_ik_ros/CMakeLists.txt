cmake_minimum_required(VERSION 3.16)
project(openarm_ik_ros LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(openarm_description REQUIRED)
find_package(openarm_model CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

option(OPENARM_IK_ROS_COVERAGE "Instrument authored adapter code for gcov" OFF)

add_library(openarm_ik_transaction STATIC src/paired_transaction.cpp)
target_include_directories(openarm_ik_transaction PRIVATE include)
target_compile_options(openarm_ik_transaction PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_ik_transaction PUBLIC openarm_model::openarm_model)

add_executable(openarm_ik_ros_node src/openarm_ik_ros_node.cpp)
target_include_directories(openarm_ik_ros_node PRIVATE include)
target_compile_options(openarm_ik_ros_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_ik_ros_node PRIVATE
  openarm_ik_transaction
  diagnostic_msgs::diagnostic_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs)

if(OPENARM_IK_ROS_COVERAGE)
  foreach(target openarm_ik_transaction openarm_ik_ros_node)
    target_compile_options(${target} PRIVATE --coverage -O0)
    target_link_options(${target} PRIVATE --coverage)
  endforeach()
endif()

install(TARGETS openarm_ik_ros_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES "${openarm_model_GENERATED_URDF}" DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_paired_transaction test/test_paired_transaction.cpp)
  target_link_libraries(test_paired_transaction openarm_ik_transaction)
  target_include_directories(test_paired_transaction PRIVATE include)
  target_compile_options(test_paired_transaction PRIVATE -Wall -Wextra -Wpedantic -Werror)
  if(OPENARM_IK_ROS_COVERAGE)
    target_compile_options(test_paired_transaction PRIVATE --coverage -O0)
    target_link_options(test_paired_transaction PRIVATE --coverage)
  endif()
  get_filename_component(OPENARM_DESCRIPTION_SHARE "${openarm_description_DIR}/.." ABSOLUTE)
  add_test(
    NAME test_generated_urdf
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generated_urdf.py
      --urdf ${openarm_model_GENERATED_URDF}
      --description-root ${OPENARM_DESCRIPTION_SHARE})
  add_test(
    NAME test_no_can_linkage
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_no_can_linkage.py
      --executable $<TARGET_FILE:openarm_ik_ros_node>)
  add_test(
    NAME test_ros_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_contract.py)
  add_test(
    NAME test_invalid_expiry_parameter
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_invalid_expiry_parameter.py)
endif()

ament_package()
