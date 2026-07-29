cmake_minimum_required(VERSION 3.16)
project(openarm_control VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)

option(OA_CONTROL_BUILD_TESTS "Build the compiled control tests" ON)
option(OA_CONTROL_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(OA_CONTROL_TSAN "Enable ThreadSanitizer" OFF)

if(OA_CONTROL_SANITIZERS AND OA_CONTROL_TSAN)
    message(FATAL_ERROR "OA_CONTROL_SANITIZERS and OA_CONTROL_TSAN are mutually exclusive")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT TARGET openarm_model)
    set(OA_MODEL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(OA_MODEL_SANITIZERS ${OA_CONTROL_SANITIZERS} CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../model"
                     "${CMAKE_CURRENT_BINARY_DIR}/openarm_model" EXCLUDE_FROM_ALL)
endif()

add_library(openarm_control STATIC
    src/c_api.cpp
    src/control_core.cpp
    src/kinematics.cpp
)
target_include_directories(openarm_control
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(openarm_control PUBLIC openarm_model)
target_compile_features(openarm_control PUBLIC c_std_11 cxx_std_17)
target_compile_options(openarm_control PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Werror;-Wshadow;-Wformat=2>
)

if(OA_CONTROL_SANITIZERS)
    target_compile_options(openarm_control PRIVATE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(openarm_control PUBLIC -fsanitize=address,undefined)
endif()
if(OA_CONTROL_TSAN)
    target_compile_options(openarm_control PRIVATE
        -fsanitize=thread -fno-omit-frame-pointer)
    target_link_options(openarm_control PUBLIC -fsanitize=thread)
endif()

install(TARGETS openarm_control
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES include/openarm_control.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CTest)
if(BUILD_TESTING AND OA_CONTROL_BUILD_TESTS)
    find_package(Threads REQUIRED)

    set(OA_PUBLIC_HEADERS
        openarm_can.h
        openarm_model.h
        openarm_transport.h
        openarm_commission.h
        openarm_control.h)
    set(OA_PUBLIC_INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/../can/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../model/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../transport/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/../commission/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/public_header_matrix")
    foreach(first_header IN LISTS OA_PUBLIC_HEADERS)
        foreach(second_header IN LISTS OA_PUBLIC_HEADERS)
            if(NOT first_header STREQUAL second_header)
                string(REPLACE ".h" "" first_name "${first_header}")
                string(REPLACE ".h" "" second_name "${second_header}")
                foreach(language IN ITEMS c cxx)
                    if(language STREQUAL c)
                        set(extension c)
                        set(standard c_std_11)
                    else()
                        set(extension cpp)
                        set(standard cxx_std_17)
                    endif()
                    foreach(legacy_mode IN ITEMS legacy canonical)
                        set(matrix_target
                            "oa_headers_${first_name}_${second_name}_${language}_${legacy_mode}")
                        set(matrix_source
                            "${CMAKE_CURRENT_BINARY_DIR}/public_header_matrix/${matrix_target}.${extension}")
                        if(legacy_mode STREQUAL canonical)
                            set(status_prefix
                                "#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1\n")
                        else()
                            set(status_prefix "")
                        endif()
                        file(GENERATE OUTPUT "${matrix_source}" CONTENT
                            "${status_prefix}#include <${first_header}>\n#include <${second_header}>\n")
                        add_library(${matrix_target} OBJECT "${matrix_source}")
                        target_include_directories(${matrix_target} PRIVATE
                            ${OA_PUBLIC_INCLUDE_DIRS})
                        target_compile_features(${matrix_target} PRIVATE ${standard})
                        set_target_properties(${matrix_target} PROPERTIES
                            C_EXTENSIONS OFF CXX_EXTENSIONS OFF)
                        target_compile_options(${matrix_target} PRIVATE
                            -Wall -Wextra -Wpedantic -Werror)
                    endforeach()
                endforeach()
            endif()
        endforeach()
    endforeach()

    add_library(openarm_public_headers_c11 OBJECT tests/public_headers_c11.c)
    target_include_directories(openarm_public_headers_c11 PRIVATE
        ${OA_PUBLIC_INCLUDE_DIRS})
    target_compile_features(openarm_public_headers_c11 PRIVATE c_std_11)
    set_target_properties(openarm_public_headers_c11 PROPERTIES C_EXTENSIONS OFF)
    target_compile_options(openarm_public_headers_c11 PRIVATE
        -Wall -Wextra -Wpedantic -Werror)

    add_library(openarm_public_headers_cxx17 OBJECT tests/public_headers_cxx17.cpp)
    target_include_directories(openarm_public_headers_cxx17 PRIVATE
        ${OA_PUBLIC_INCLUDE_DIRS})
    target_compile_features(openarm_public_headers_cxx17 PRIVATE cxx_std_17)
    set_target_properties(openarm_public_headers_cxx17 PROPERTIES CXX_EXTENSIONS OFF)
    target_compile_options(openarm_public_headers_cxx17 PRIVATE
        -Wall -Wextra -Wpedantic -Werror)

    add_executable(openarm_control_tests tests/test_control.cpp)
    target_link_libraries(openarm_control_tests PRIVATE openarm_control Threads::Threads)
    target_compile_options(openarm_control_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2)
    if(OA_CONTROL_SANITIZERS OR OA_CONTROL_TSAN)
        target_compile_definitions(openarm_control_tests PRIVATE
            OA_CONTROL_TEST_SANITIZED=1)
    endif()
    add_test(NAME openarm_control_tests COMMAND openarm_control_tests)

    add_executable(openarm_control_c11_abi tests/c11_abi_consumer.c)
    target_link_libraries(openarm_control_c11_abi PRIVATE openarm_control)
    set_target_properties(openarm_control_c11_abi PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_options(openarm_control_c11_abi PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2)
    add_test(NAME openarm_control_c11_abi COMMAND openarm_control_c11_abi)

    add_executable(openarm_control_v1_original_abi tests/v1_original_abi_consumer.c)
    target_include_directories(openarm_control_v1_original_abi BEFORE PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1_original")
    target_link_libraries(openarm_control_v1_original_abi PRIVATE openarm_control)
    set_target_properties(openarm_control_v1_original_abi PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_options(openarm_control_v1_original_abi PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2)
    add_test(NAME openarm_control_v1_original_abi COMMAND openarm_control_v1_original_abi)

    if(NOT OA_CONTROL_SANITIZERS AND NOT OA_CONTROL_TSAN)
        add_test(NAME openarm_public_headers_install_consumer
            COMMAND "${CMAKE_COMMAND}"
                -DOA_CONTROL_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -DOA_MODEL_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/openarm_model
                -DOA_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_install_consumer.cmake")
    endif()
endif()
