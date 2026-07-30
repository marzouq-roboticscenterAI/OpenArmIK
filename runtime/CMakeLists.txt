cmake_minimum_required(VERSION 3.16)
project(openarm_runtime VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(OA_RUNTIME_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(OA_RUNTIME_ENABLE_THREAD_SANITIZER "Enable ThreadSanitizer" OFF)

if(OA_RUNTIME_ENABLE_SANITIZERS AND OA_RUNTIME_ENABLE_THREAD_SANITIZER)
    message(FATAL_ERROR "Runtime sanitizer modes are mutually exclusive")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(openarm_model 1 CONFIG REQUIRED)
find_package(openarm_commission 0.1.0 EXACT CONFIG REQUIRED)
find_package(openarm_control 1 CONFIG REQUIRED)

if(NOT TARGET OpenArm::Model)
    add_library(OpenArm::Model INTERFACE IMPORTED)
    set_property(TARGET OpenArm::Model PROPERTY
        INTERFACE_LINK_LIBRARIES openarm_model::openarm_model)
endif()
set(OA_RUNTIME_SOURCES
    src/calibration.cpp
    src/inventory.cpp
    src/manifest.cpp
    src/persistence.cpp
    src/runtime.cpp
    src/sha256.cpp)

add_library(openarm_runtime STATIC ${OA_RUNTIME_SOURCES})
add_library(OpenArm::Runtime ALIAS openarm_runtime)
set_target_properties(openarm_runtime PROPERTIES EXPORT_NAME Runtime)
target_include_directories(openarm_runtime
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(openarm_runtime PUBLIC
    OpenArm::Model OpenArm::Commission OpenArm::Control Threads::Threads)
target_compile_options(openarm_runtime PRIVATE
    -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2 -Wconversion -Wsign-conversion)

if(OA_RUNTIME_ENABLE_SANITIZERS)
    target_compile_options(openarm_runtime PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(openarm_runtime PUBLIC -fsanitize=address,undefined)
elseif(OA_RUNTIME_ENABLE_THREAD_SANITIZER)
    target_compile_options(openarm_runtime PUBLIC -fsanitize=thread -fno-omit-frame-pointer)
    target_link_options(openarm_runtime PUBLIC -fsanitize=thread)
endif()

if(BUILD_TESTING)
    add_library(openarm_runtime_test STATIC ${OA_RUNTIME_SOURCES})
    target_include_directories(openarm_runtime_test
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(openarm_runtime_test PUBLIC
        OpenArm::Model OpenArm::Commission OpenArm::Control Threads::Threads)
    target_compile_definitions(openarm_runtime_test PRIVATE OA_RUNTIME_ENABLE_TEST_HOOKS=1)
    target_compile_options(openarm_runtime_test PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2 -Wconversion -Wsign-conversion)
    if(OA_RUNTIME_ENABLE_SANITIZERS)
        target_compile_options(openarm_runtime_test PUBLIC
            -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(openarm_runtime_test PUBLIC -fsanitize=address,undefined)
    elseif(OA_RUNTIME_ENABLE_THREAD_SANITIZER)
        target_compile_options(openarm_runtime_test PUBLIC
            -fsanitize=thread -fno-omit-frame-pointer)
        target_link_options(openarm_runtime_test PUBLIC -fsanitize=thread)
    endif()

    add_executable(openarm_runtime_tests tests/test_runtime.cpp)
    target_link_libraries(openarm_runtime_tests PRIVATE openarm_runtime_test)
    target_compile_options(openarm_runtime_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2)
    add_test(NAME openarm_runtime_tests COMMAND openarm_runtime_tests)

    add_executable(openarm_runtime_c11 tests/c11_consumer.c)
    target_link_libraries(openarm_runtime_c11 PRIVATE openarm_runtime)
    set_target_properties(openarm_runtime_c11 PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_options(openarm_runtime_c11 PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME openarm_runtime_c11 COMMAND openarm_runtime_c11)

    add_executable(openarm_runtime_v1_c11 tests/abi_v1_consumer.c)
    target_include_directories(openarm_runtime_v1_c11 BEFORE PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1")
    target_link_libraries(openarm_runtime_v1_c11 PRIVATE openarm_runtime)
    set_target_properties(openarm_runtime_v1_c11 PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_options(openarm_runtime_v1_c11 PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME openarm_runtime_v1_c11 COMMAND openarm_runtime_v1_c11)

    add_executable(openarm_runtime_v1_cxx17 tests/abi_v1_consumer.cpp)
    target_include_directories(openarm_runtime_v1_cxx17 BEFORE PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1")
    target_link_libraries(openarm_runtime_v1_cxx17 PRIVATE openarm_runtime)
    target_compile_options(openarm_runtime_v1_cxx17 PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME openarm_runtime_v1_cxx17 COMMAND openarm_runtime_v1_cxx17)

    add_executable(openarm_runtime_v1_current_c11 tests/abi_v1_consumer.c)
    target_include_directories(openarm_runtime_v1_current_c11 BEFORE PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_include_directories(openarm_runtime_v1_current_c11 PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1")
    target_link_libraries(openarm_runtime_v1_current_c11 PRIVATE openarm_runtime)
    set_target_properties(openarm_runtime_v1_current_c11 PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_options(openarm_runtime_v1_current_c11 PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME openarm_runtime_v1_current_c11 COMMAND openarm_runtime_v1_current_c11)

    add_executable(openarm_runtime_v1_current_cxx17 tests/abi_v1_consumer.cpp)
    target_include_directories(openarm_runtime_v1_current_cxx17 BEFORE PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_include_directories(openarm_runtime_v1_current_cxx17 PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1")
    target_link_libraries(openarm_runtime_v1_current_cxx17 PRIVATE openarm_runtime)
    target_compile_options(openarm_runtime_v1_current_cxx17 PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME openarm_runtime_v1_current_cxx17 COMMAND openarm_runtime_v1_current_cxx17)

    add_test(NAME openarm_runtime_v1_header_policy
        COMMAND "${CMAKE_COMMAND}"
            -DOA_CURRENT_HEADER=${CMAKE_CURRENT_SOURCE_DIR}/include/openarm_runtime.h
            -DOA_FROZEN_HEADER=${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1/openarm_runtime.h
            -DOA_CURRENT_COMMISSION_HEADER=${CMAKE_CURRENT_SOURCE_DIR}/../commission/include/openarm_commission.h
            -DOA_FROZEN_COMMISSION_HEADER=${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1/openarm_commission.h
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_abi_v1_header.cmake)
    add_test(NAME openarm_runtime_v1_symbol_manifest
        COMMAND "${CMAKE_COMMAND}"
            -DOA_NM=${CMAKE_NM}
            -DOA_RUNTIME_ARCHIVE=$<TARGET_FILE:openarm_runtime>
            -DOA_EXPECTED_SYMBOLS=${CMAKE_CURRENT_SOURCE_DIR}/tests/abi_v1/expected_symbols.txt
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_symbol_manifest.cmake)
    get_filename_component(OA_RUNTIME_MODEL_CMAKE_ROOT "${openarm_model_DIR}" DIRECTORY)
    get_filename_component(OA_RUNTIME_MODEL_LIB_ROOT "${OA_RUNTIME_MODEL_CMAKE_ROOT}" DIRECTORY)
    get_filename_component(OA_RUNTIME_DEPENDENCY_PREFIX "${OA_RUNTIME_MODEL_LIB_ROOT}" DIRECTORY)
    add_test(NAME openarm_runtime_v1_install_consumer
        COMMAND "${CMAKE_COMMAND}"
            -DOA_RUNTIME_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -DOA_RUNTIME_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -DOA_DEPENDENCY_PREFIX=${OA_RUNTIME_DEPENDENCY_PREFIX}
            -DOA_NM=${CMAKE_NM}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_install_consumer.cmake)
endif()

install(TARGETS openarm_runtime EXPORT openarm_runtimeTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES include/openarm_runtime.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES tests/abi_v1/openarm_runtime.h tests/abi_v1/openarm_commission.h
    tests/abi_v1/layout_contract.h tests/abi_v1/symbol_references.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/openarm_runtime_abi_v1)
install(EXPORT openarm_runtimeTargets
    FILE openarm_runtimeTargets.cmake
    NAMESPACE OpenArm::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_runtime)
configure_package_config_file(
    cmake/openarm_runtimeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_runtimeConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_runtime)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_runtimeConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_runtimeConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/openarm_runtimeConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_runtime)
