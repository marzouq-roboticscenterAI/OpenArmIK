cmake_minimum_required(VERSION 3.16)
project(openarm_transport VERSION 1.0.0 LANGUAGES C CXX)

option(OA_TRANSPORT_ENABLE_SANITIZERS
       "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(OA_TRANSPORT_ENABLE_THREAD_SANITIZER
       "Enable ThreadSanitizer" OFF)
option(OA_TRANSPORT_BUILD_VCAN_SMOKE
       "Build and register the vcan0 smoke test" ON)

if(OA_TRANSPORT_ENABLE_SANITIZERS AND OA_TRANSPORT_ENABLE_THREAD_SANITIZER)
    message(FATAL_ERROR "ASan/UBSan and TSan cannot be enabled together")
endif()

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

function(oa_transport_strict_c target)
    target_compile_features(${target} PRIVATE c_std_11)
    set_target_properties(${target} PROPERTIES
        C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX)
    elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
            -Wshadow -Wformat=2 -Wundef -Wcast-align -Wstrict-prototypes
            -Wmissing-prototypes)
        if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
            target_compile_options(${target} PRIVATE -Wcast-align=strict)
        endif()
    endif()
endfunction()

function(oa_transport_strict_cxx target)
    target_compile_features(${target} PRIVATE cxx_std_17)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
            -Wshadow -Wformat=2 -Wundef -Wcast-align -Wnon-virtual-dtor)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            target_compile_options(${target} PRIVATE -Wcast-align=strict)
        endif()
    endif()
endfunction()

function(oa_transport_sanitizers target)
    if(OA_TRANSPORT_ENABLE_SANITIZERS AND
       CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    elseif(OA_TRANSPORT_ENABLE_THREAD_SANITIZER AND
           CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -fsanitize=thread -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=thread)
    endif()
endfunction()

add_library(openarm_transport_codec OBJECT
    "${CMAKE_CURRENT_SOURCE_DIR}/../can/src/openarm_can.c")
target_include_directories(openarm_transport_codec PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../can/include")
oa_transport_strict_c(openarm_transport_codec)
if(OA_TRANSPORT_ENABLE_SANITIZERS AND CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(openarm_transport_codec PRIVATE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
elseif(OA_TRANSPORT_ENABLE_THREAD_SANITIZER AND
       CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(openarm_transport_codec PRIVATE
        -fsanitize=thread -fno-omit-frame-pointer)
endif()

add_library(openarm_transport STATIC
    src/c_api.cpp
    src/socketcan_backend.cpp
    src/transport.cpp
    $<TARGET_OBJECTS:openarm_transport_codec>)
target_include_directories(openarm_transport
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../can/include")
target_link_libraries(openarm_transport PUBLIC Threads::Threads)
oa_transport_strict_cxx(openarm_transport)
oa_transport_sanitizers(openarm_transport)

include(CTest)
if(BUILD_TESTING)
    add_executable(openarm_transport_tests tests/test_transport.cpp)
    target_include_directories(openarm_transport_tests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(openarm_transport_tests PRIVATE openarm_transport)
    oa_transport_strict_cxx(openarm_transport_tests)
    oa_transport_sanitizers(openarm_transport_tests)
    add_test(NAME openarm_transport_tests COMMAND openarm_transport_tests)

    add_executable(openarm_transport_c11_consumer tests/c11_consumer.c)
    target_link_libraries(openarm_transport_c11_consumer PRIVATE openarm_transport)
    set_target_properties(openarm_transport_c11_consumer PROPERTIES
        LINKER_LANGUAGE CXX)
    oa_transport_strict_c(openarm_transport_c11_consumer)
    oa_transport_sanitizers(openarm_transport_c11_consumer)
    add_test(NAME openarm_transport_c11_consumer
             COMMAND openarm_transport_c11_consumer)

    if(OA_TRANSPORT_BUILD_VCAN_SMOKE)
        add_executable(openarm_transport_vcan_smoke tests/vcan_smoke.cpp)
        target_link_libraries(openarm_transport_vcan_smoke PRIVATE openarm_transport)
        oa_transport_strict_cxx(openarm_transport_vcan_smoke)
        oa_transport_sanitizers(openarm_transport_vcan_smoke)
        add_test(NAME openarm_transport_vcan_smoke
                 COMMAND openarm_transport_vcan_smoke)
    endif()

    if(NOT OA_TRANSPORT_ENABLE_SANITIZERS AND
       NOT OA_TRANSPORT_ENABLE_THREAD_SANITIZER)
        add_test(NAME openarm_transport_install_consumer
            COMMAND "${CMAKE_COMMAND}"
                -DOA_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -DOA_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_install_consumer.cmake")
    endif()
endif()

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/OpenArmTransportConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenArmTransportConfig.cmake"
    INSTALL_DESTINATION lib/cmake/OpenArmTransport)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenArmTransportConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(TARGETS openarm_transport EXPORT OpenArmTransportTargets
        ARCHIVE DESTINATION lib)
install(FILES include/openarm_transport.h DESTINATION include)
install(EXPORT OpenArmTransportTargets
        FILE OpenArmTransportTargets.cmake
        NAMESPACE OpenArm::
        DESTINATION lib/cmake/OpenArmTransport)
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/OpenArmTransportConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/OpenArmTransportConfigVersion.cmake"
        DESTINATION lib/cmake/OpenArmTransport)
