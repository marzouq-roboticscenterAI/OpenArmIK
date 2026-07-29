cmake_minimum_required(VERSION 3.16)
project(openarm_public_headers_install_consumer LANGUAGES C CXX)

find_path(OPENARM_INSTALLED_INCLUDE_DIR
    NAMES openarm_model.h openarm_control.h
    PATHS "${CMAKE_PREFIX_PATH}/include"
    NO_DEFAULT_PATH
    REQUIRED)

add_executable(installed_headers_c11 consumer.c)
target_include_directories(installed_headers_c11 PRIVATE
    "${OPENARM_INSTALLED_INCLUDE_DIR}")
target_compile_features(installed_headers_c11 PRIVATE c_std_11)
set_target_properties(installed_headers_c11 PROPERTIES C_EXTENSIONS OFF)
target_compile_options(installed_headers_c11 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)

add_executable(installed_headers_cxx17 consumer.cpp)
target_include_directories(installed_headers_cxx17 PRIVATE
    "${OPENARM_INSTALLED_INCLUDE_DIR}")
target_compile_features(installed_headers_cxx17 PRIVATE cxx_std_17)
set_target_properties(installed_headers_cxx17 PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(installed_headers_cxx17 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
