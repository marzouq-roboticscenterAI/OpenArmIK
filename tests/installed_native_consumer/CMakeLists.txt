cmake_minimum_required(VERSION 3.16)
project(openarm_installed_native_consumer LANGUAGES C CXX)

find_package(OpenArmCan 1 CONFIG REQUIRED)
find_package(openarm_model 1 CONFIG REQUIRED)
find_package(openarm_commission CONFIG REQUIRED)
find_package(OpenArmTransport 1 CONFIG REQUIRED)
find_package(openarm_control 1 CONFIG REQUIRED)
find_package(openarm_runtime 1 CONFIG REQUIRED)

if(NOT TARGET OpenArm::Model)
  add_library(OpenArm::Model INTERFACE IMPORTED)
  set_property(TARGET OpenArm::Model PROPERTY
    INTERFACE_LINK_LIBRARIES openarm_model::openarm_model)
endif()
if(NOT TARGET OpenArm::Transport)
  add_library(OpenArm::Transport INTERFACE IMPORTED)
  set_property(TARGET OpenArm::Transport PROPERTY
    INTERFACE_LINK_LIBRARIES OpenArm::openarm_transport)
endif()

set(openarm_components
  OpenArm::Can
  OpenArm::Model
  OpenArm::Commission
  OpenArm::Transport
  OpenArm::Control
  OpenArm::Runtime)

add_executable(openarm_installed_c11 all_headers.c)
target_link_libraries(openarm_installed_c11 PRIVATE ${openarm_components})
set_target_properties(openarm_installed_c11 PROPERTIES
  C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
  LINKER_LANGUAGE CXX)

add_executable(openarm_installed_cxx17 all_headers.cpp)
target_link_libraries(openarm_installed_cxx17 PRIVATE ${openarm_components})
set_target_properties(openarm_installed_cxx17 PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

add_executable(openarm_runtime_only_c11 runtime_only.c)
target_link_libraries(openarm_runtime_only_c11 PRIVATE OpenArm::Runtime)
set_target_properties(openarm_runtime_only_c11 PROPERTIES
  C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
  LINKER_LANGUAGE CXX)

add_executable(openarm_runtime_only_cxx17 runtime_only.cpp)
target_link_libraries(openarm_runtime_only_cxx17 PRIVATE OpenArm::Runtime)
set_target_properties(openarm_runtime_only_cxx17 PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(openarm_installed_c11 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(openarm_installed_cxx17 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
  target_compile_options(openarm_runtime_only_cxx17 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
endif()
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(openarm_runtime_only_c11 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
endif()
