cmake_minimum_required(VERSION 3.16)
project(openarm_public_headers_install_consumer LANGUAGES C CXX)

find_package(openarm_model 1.0 CONFIG REQUIRED)
find_package(openarm_control 1.0 CONFIG REQUIRED)

add_executable(installed_headers_c11 consumer.c)
target_link_libraries(installed_headers_c11 PRIVATE
    openarm_control::openarm_control
    openarm_model::openarm_model)
target_compile_features(installed_headers_c11 PRIVATE c_std_11)
set_target_properties(installed_headers_c11 PROPERTIES C_EXTENSIONS OFF)
target_compile_options(installed_headers_c11 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)

add_executable(installed_headers_cxx17 consumer.cpp)
target_link_libraries(installed_headers_cxx17 PRIVATE
    openarm_control::openarm_control
    openarm_model::openarm_model)
target_compile_features(installed_headers_cxx17 PRIVATE cxx_std_17)
set_target_properties(installed_headers_cxx17 PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(installed_headers_cxx17 PRIVATE
    -Wall -Wextra -Wpedantic -Werror)
