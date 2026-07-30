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
find_package(JPEG REQUIRED)
find_package(openarm_control_msgs REQUIRED)
find_package(openarm_description REQUIRED)
find_package(openarm_model CONFIG REQUIRED)
find_package(openarm_runtime CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)
find_package(X11 REQUIRED)
if(NOT X11_Xcomposite_FOUND)
  message(FATAL_ERROR "XComposite development files are required")
endif()

option(OPENARM_IK_ROS_COVERAGE "Instrument authored adapter code for gcov" OFF)

set(OPENARM_STAGE_A_VISUALIZATION_URDF
  "${CMAKE_CURRENT_BINARY_DIR}/generated/openarm_v10_bimanual_stage_a_visualization.urdf")
add_custom_command(
  OUTPUT "${OPENARM_STAGE_A_VISUALIZATION_URDF}"
  COMMAND "${CMAKE_COMMAND}"
    "-DINPUT_URDF=${openarm_model_GENERATED_URDF}"
    "-DOUTPUT_URDF=${OPENARM_STAGE_A_VISUALIZATION_URDF}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateStageAVisualizationUrdf.cmake"
  DEPENDS
    "${openarm_model_GENERATED_URDF}"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateStageAVisualizationUrdf.cmake"
  COMMENT "Deriving the Stage-A visualization-only URDF"
  VERBATIM)
add_custom_target(openarm_stage_a_visualization_urdf ALL
  DEPENDS "${OPENARM_STAGE_A_VISUALIZATION_URDF}")

add_library(openarm_virtual_control_session STATIC src/virtual_control_session.cpp)
target_include_directories(openarm_virtual_control_session PUBLIC include)
target_compile_options(openarm_virtual_control_session PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_virtual_control_session PUBLIC
  OpenArm::Runtime Threads::Threads)

add_executable(openarm_ik_ros_node src/openarm_ik_ros_node.cpp)
target_include_directories(openarm_ik_ros_node PRIVATE include)
target_compile_options(openarm_ik_ros_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_ik_ros_node PRIVATE
  openarm_virtual_control_session
  diagnostic_msgs::diagnostic_msgs
  geometry_msgs::geometry_msgs
  openarm_control_msgs::openarm_control_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros)

add_executable(openarm_control_cli src/openarm_control_cli.cpp)
target_compile_options(openarm_control_cli PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_control_cli PRIVATE
  diagnostic_msgs::diagnostic_msgs
  openarm_control_msgs::openarm_control_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action)

add_executable(close_rviz_window src/close_rviz_window.cpp)
target_compile_options(close_rviz_window PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(close_rviz_window PRIVATE X11::X11)

add_library(openarm_portal_core STATIC src/portal_core.cpp src/portal_page.cpp)
target_include_directories(openarm_portal_core PUBLIC include)
target_compile_options(openarm_portal_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_portal_core PUBLIC openarm_model::openarm_model)

add_executable(openarm_portal src/openarm_portal.cpp src/rviz_capture.cpp)
target_include_directories(openarm_portal PRIVATE include ${X11_Xcomposite_INCLUDE_PATH})
target_compile_definitions(openarm_portal PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_compile_options(openarm_portal PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_portal PRIVATE
  openarm_portal_core
  diagnostic_msgs::diagnostic_msgs
  openarm_control_msgs::openarm_control_msgs
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  sensor_msgs::sensor_msgs
  X11::X11
  ${X11_Xcomposite_LIB}
  JPEG::JPEG
  Threads::Threads)

if(OPENARM_IK_ROS_COVERAGE)
  foreach(target openarm_virtual_control_session openarm_ik_ros_node openarm_control_cli openarm_portal_core openarm_portal)
    target_compile_options(${target} PRIVATE --coverage -O0)
    target_link_options(${target} PRIVATE --coverage)
  endforeach()
endif()

install(TARGETS openarm_ik_ros_node openarm_control_cli close_rviz_window openarm_portal
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES package.xml README.md DESTINATION share/${PROJECT_NAME})
install(FILES "${openarm_model_GENERATED_URDF}" DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES "${OPENARM_STAGE_A_VISUALIZATION_URDF}" DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  target_compile_definitions(openarm_virtual_control_session PUBLIC OPENARM_IK_ROS_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_virtual_control_session test/test_virtual_control_session.cpp)
  target_link_libraries(test_virtual_control_session openarm_virtual_control_session)
  target_include_directories(test_virtual_control_session PRIVATE include)
  target_compile_options(test_virtual_control_session PRIVATE -Wall -Wextra -Wpedantic -Werror)
  ament_add_gtest(test_portal_core test/test_portal_core.cpp)
  target_link_libraries(test_portal_core
    openarm_portal_core
    openarm_control_msgs::openarm_control_msgs)
  target_include_directories(test_portal_core PRIVATE include)
  target_compile_options(test_portal_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
  if(OPENARM_IK_ROS_COVERAGE)
    target_compile_options(test_virtual_control_session PRIVATE --coverage -O0)
    target_link_options(test_virtual_control_session PRIVATE --coverage)
  endif()
  add_executable(test_openarm_control_cli src/openarm_control_cli.cpp)
  target_compile_definitions(test_openarm_control_cli PRIVATE
    OPENARM_CLI_RESULT_TIMEOUT_MS=1500
    OPENARM_CLI_CANCEL_TIMEOUT_MS=500
    OPENARM_CLI_CANCEL_RESULT_TIMEOUT_MS=1000
    OPENARM_CLI_TEST_CONTEXT_SHUTDOWN_AFTER_ACCEPT)
  target_compile_options(test_openarm_control_cli PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(test_openarm_control_cli PRIVATE
    diagnostic_msgs::diagnostic_msgs
    openarm_control_msgs::openarm_control_msgs
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action)
  if(OPENARM_IK_ROS_COVERAGE)
    target_compile_options(test_openarm_control_cli PRIVATE --coverage -O0)
    target_link_options(test_openarm_control_cli PRIVATE --coverage)
  endif()
  add_test(
    NAME test_cli_server_lifecycle
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cli_server_lifecycle.py
      --executable $<TARGET_FILE:test_openarm_control_cli>
      --production-executable $<TARGET_FILE:openarm_control_cli>)
  add_test(
    NAME test_no_can_linkage
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_no_can_linkage.py
      --executable $<TARGET_FILE:openarm_ik_ros_node>
      --session-library $<TARGET_FILE:openarm_virtual_control_session>)
  get_filename_component(OPENARM_DESCRIPTION_SHARE "${openarm_description_DIR}/.." ABSOLUTE)
  add_test(
    NAME test_generated_urdf
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generated_urdf.py
      --urdf ${openarm_model_GENERATED_URDF}
      --description-root ${OPENARM_DESCRIPTION_SHARE})
  add_test(
    NAME test_visualization_urdf
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_visualization_urdf.py
      --canonical ${openarm_model_GENERATED_URDF}
      --visualization ${OPENARM_STAGE_A_VISUALIZATION_URDF}
      --generator ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateStageAVisualizationUrdf.cmake
      --cmake ${CMAKE_COMMAND})
  find_program(CHECK_URDF_EXECUTABLE NAMES check_urdf REQUIRED)
  add_test(NAME test_visualization_urdf_parser
    COMMAND ${CHECK_URDF_EXECUTABLE} ${OPENARM_STAGE_A_VISUALIZATION_URDF})
  add_test(
    NAME test_invalid_expiry_parameter
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_invalid_expiry_parameter.py)
  add_test(
    NAME test_ros_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_contract.py)
  add_test(
    NAME test_active_sigint
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_active_sigint.py)
  set_tests_properties(test_ros_contract test_active_sigint test_cli_server_lifecycle
    test_no_can_linkage test_invalid_expiry_parameter
    PROPERTIES RUN_SERIAL TRUE)
  add_test(NAME test_close_rviz_window_help COMMAND close_rviz_window --help)
  add_test(NAME test_close_rviz_window_invalid_pid COMMAND close_rviz_window 0)
  add_test(NAME test_close_rviz_window_invalid_timeout COMMAND close_rviz_window 1 --timeout infinity)
  set_tests_properties(test_close_rviz_window_invalid_pid test_close_rviz_window_invalid_timeout
    PROPERTIES WILL_FAIL TRUE)
endif()

ament_package()
