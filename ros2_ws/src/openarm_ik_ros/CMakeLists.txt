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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(OPENARM_MODEL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../model")
set(OPENARM_GENERATED_URDF "${OPENARM_MODEL_ROOT}/generated/openarm_v10_bimanual.urdf")
if(NOT EXISTS "${OPENARM_MODEL_ROOT}/src/openarm_model.c" OR NOT EXISTS "${OPENARM_GENERATED_URDF}")
  message(FATAL_ERROR "The checked-in pure C model and generated bimanual URDF are required.")
endif()

add_library(openarm_ik_model STATIC "${OPENARM_MODEL_ROOT}/src/openarm_model.c")
target_include_directories(openarm_ik_model PUBLIC "${OPENARM_MODEL_ROOT}/include")
target_compile_options(openarm_ik_model PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_ik_model PUBLIC m)

add_library(openarm_ik_transaction STATIC src/paired_transaction.cpp)
target_include_directories(openarm_ik_transaction PUBLIC include "${OPENARM_MODEL_ROOT}/include")
target_compile_options(openarm_ik_transaction PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_ik_transaction PUBLIC openarm_ik_model)

add_executable(openarm_ik_ros_node src/openarm_ik_ros_node.cpp)
target_compile_options(openarm_ik_ros_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_ik_ros_node PRIVATE
  openarm_ik_transaction
  diagnostic_msgs::diagnostic_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs)

install(TARGETS openarm_ik_ros_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES "${OPENARM_GENERATED_URDF}" DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES "${OPENARM_MODEL_ROOT}/LICENSE.txt" "${OPENARM_MODEL_ROOT}/NOTICE"
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_paired_transaction test/test_paired_transaction.cpp)
  target_link_libraries(test_paired_transaction openarm_ik_transaction)
  target_include_directories(test_paired_transaction PRIVATE include "${OPENARM_MODEL_ROOT}/include")
  target_compile_options(test_paired_transaction PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(
    NAME test_generated_urdf
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generated_urdf.py
      --urdf ${OPENARM_GENERATED_URDF}
      --description-root ${CMAKE_CURRENT_SOURCE_DIR}/../../../upstream/openarm_description)
  add_test(
    NAME test_no_can_linkage
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_no_can_linkage.py
      --executable $<TARGET_FILE:openarm_ik_ros_node>)
endif()

ament_package()
