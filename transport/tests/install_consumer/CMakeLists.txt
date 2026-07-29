cmake_minimum_required(VERSION 3.16)
project(openarm_transport_installed_consumer LANGUAGES C CXX)

find_package(OpenArmTransport 1 CONFIG REQUIRED)

add_executable(openarm_transport_installed_consumer ../c11_consumer.c)
target_link_libraries(openarm_transport_installed_consumer
    PRIVATE OpenArm::openarm_transport)
set_target_properties(openarm_transport_installed_consumer PROPERTIES
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
    LINKER_LANGUAGE CXX)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(openarm_transport_installed_consumer PRIVATE
        -Wall -Wextra -Wpedantic -Werror)
endif()
