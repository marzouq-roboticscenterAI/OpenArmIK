cmake_minimum_required(VERSION 3.16)
project(openarm_control VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

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

find_package(Threads REQUIRED)

if(NOT TARGET OpenArm::Model AND NOT TARGET openarm_model::openarm_model)
    find_package(openarm_model 1 CONFIG REQUIRED)
endif()
if(TARGET OpenArm::Model)
    set(OPENARM_CONTROL_MODEL_TARGET OpenArm::Model)
else()
    set(OPENARM_CONTROL_MODEL_TARGET openarm_model::openarm_model)
endif()

set(OPENARM_CONTROL_SOURCES
    src/c_api.cpp
    src/control_core.cpp
    src/kinematics.cpp)

function(openarm_configure_control_target target)
    target_include_directories(${target}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(${target} PUBLIC
        ${OPENARM_CONTROL_MODEL_TARGET}
        Threads::Threads)
    target_compile_features(${target} PUBLIC c_std_11 cxx_std_17)
    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-Werror;-Wshadow;-Wformat=2>)
    if(OA_CONTROL_SANITIZERS)
        target_compile_options(${target} PRIVATE
            -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${target} PUBLIC -fsanitize=address,undefined)
    endif()
    if(OA_CONTROL_TSAN)
        target_compile_options(${target} PRIVATE
            -fsanitize=thread -fno-omit-frame-pointer)
        target_link_options(${target} PUBLIC -fsanitize=thread)
    endif()
endfunction()

add_library(openarm_control STATIC ${OPENARM_CONTROL_SOURCES})
add_library(openarm_control::openarm_control ALIAS openarm_control)
add_library(OpenArm::Control ALIAS openarm_control)
openarm_configure_control_target(openarm_control)
set_target_properties(openarm_control PROPERTIES EXPORT_NAME openarm_control)

install(TARGETS openarm_control EXPORT openarm_controlTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES include/openarm_control.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT openarm_controlTargets
    FILE openarm_controlTargets.cmake
    NAMESPACE openarm_control::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_control)
configure_package_config_file(
    cmake/openarm_controlConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_controlConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_control)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_controlConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_controlConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_controlConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_control)

include(CTest)
if(BUILD_TESTING AND OA_CONTROL_BUILD_TESTS)
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
                        if(legacy_mode STREQUAL legacy AND
                           ((first_header STREQUAL "openarm_model.h" AND
                             second_header STREQUAL "openarm_control.h") OR
                            (first_header STREQUAL "openarm_control.h" AND
                             second_header STREQUAL "openarm_model.h")))
                            continue()
                        endif()
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

    include(CheckCSourceCompiles)
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_INCLUDES ${OA_PUBLIC_INCLUDE_DIRS})
    set(CMAKE_REQUIRED_FLAGS "-pedantic-errors -Wall -Wextra -Werror")
    set(model_control_source
        "#ifdef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#undef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#endif\n#include <openarm_model.h>\n#include <openarm_control.h>\nint main(void) { return 0; }")
    set(control_model_source
        "#ifdef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#undef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#endif\n#include <openarm_control.h>\n#include <openarm_model.h>\nint main(void) { return 0; }")
    set(model_late_optout_source
        "#ifdef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#undef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#endif\n#include <openarm_model.h>\n#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1\n#include <openarm_control.h>\nint main(void) { return 0; }")
    set(control_late_optout_source
        "#ifdef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#undef OPENARM_DISABLE_LEGACY_GENERIC_STATUS\n#endif\n#include <openarm_control.h>\n#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1\n#include <openarm_model.h>\nint main(void) { return 0; }")
    foreach(order IN ITEMS model_control control_model model_late_optout control_late_optout)
        check_c_source_compiles("${${order}_source}"
            oa_${order}_legacy_c11_compiles)
        if(oa_${order}_legacy_c11_compiles)
            message(FATAL_ERROR
                "${order} unexpectedly accepted ambiguous legacy status names in C11")
        endif()
        check_cxx_source_compiles("${${order}_source}"
            oa_${order}_legacy_cxx17_compiles)
        if(oa_${order}_legacy_cxx17_compiles)
            message(FATAL_ERROR
                "${order} unexpectedly accepted ambiguous legacy status names in C++17")
        endif()
    endforeach()
    unset(CMAKE_REQUIRED_FLAGS)
    unset(CMAKE_REQUIRED_INCLUDES)

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

    add_library(openarm_legacy_control_status OBJECT tests/legacy_control_status.c)
    target_include_directories(openarm_legacy_control_status PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_compile_features(openarm_legacy_control_status PRIVATE c_std_11)
    set_target_properties(openarm_legacy_control_status PROPERTIES C_EXTENSIONS OFF)
    target_compile_options(openarm_legacy_control_status PRIVATE
        -Wall -Wextra -Wpedantic -Werror)

    add_library(openarm_legacy_model_status OBJECT tests/legacy_model_status.c)
    target_include_directories(openarm_legacy_model_status PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../model/include")
    target_compile_features(openarm_legacy_model_status PRIVATE c_std_11)
    set_target_properties(openarm_legacy_model_status PROPERTIES C_EXTENSIONS OFF)
    target_compile_options(openarm_legacy_model_status PRIVATE
        -Wall -Wextra -Wpedantic -Werror)

    add_library(openarm_control_test STATIC ${OPENARM_CONTROL_SOURCES})
    target_compile_definitions(openarm_control_test PRIVATE
        OPENARM_CONTROL_ENABLE_TEST_HOOKS=1)
    openarm_configure_control_target(openarm_control_test)
    add_executable(openarm_control_tests tests/test_control.cpp)
    target_include_directories(openarm_control_tests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_link_libraries(openarm_control_tests PRIVATE
        openarm_control_test Threads::Threads)
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
        get_filename_component(OPENARM_CONTROL_MODEL_CMAKE_ROOT
            "${openarm_model_DIR}" DIRECTORY)
        get_filename_component(OPENARM_CONTROL_MODEL_LIB_ROOT
            "${OPENARM_CONTROL_MODEL_CMAKE_ROOT}" DIRECTORY)
        get_filename_component(OPENARM_CONTROL_MODEL_PREFIX
            "${OPENARM_CONTROL_MODEL_LIB_ROOT}" DIRECTORY)
        add_test(NAME openarm_public_headers_install_consumer
            COMMAND "${CMAKE_COMMAND}"
                -DOA_CONTROL_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -DOA_MODEL_PREFIX=${OPENARM_CONTROL_MODEL_PREFIX}
                -DOA_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_install_consumer.cmake")
    endif()
endif()
