cmake_minimum_required(VERSION 3.16)
project(openarm_transport VERSION 1.0.0 LANGUAGES C CXX)

option(OA_TRANSPORT_ENABLE_SANITIZERS
       "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

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
    endif()
endfunction()

add_library(openarm_transport STATIC
    src/c_api.cpp
    src/socketcan_backend.cpp
    src/transport.cpp)
target_include_directories(openarm_transport
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
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

    add_executable(openarm_transport_vcan_smoke tests/vcan_smoke.cpp)
    target_link_libraries(openarm_transport_vcan_smoke PRIVATE openarm_transport)
    oa_transport_strict_cxx(openarm_transport_vcan_smoke)
    oa_transport_sanitizers(openarm_transport_vcan_smoke)
    add_test(NAME openarm_transport_vcan_smoke
             COMMAND openarm_transport_vcan_smoke)
endif()

install(TARGETS openarm_transport ARCHIVE DESTINATION lib)
install(FILES include/openarm_transport.h DESTINATION include)
