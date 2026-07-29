cmake_minimum_required(VERSION 3.20)
project(openarm_commission VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
include(CMakePackageConfigHelpers)

option(OA_COMMISSION_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" ON)

add_library(openarm_commission STATIC
    src/calibration_session.cpp
    src/c_api.cpp)
add_library(OpenArm::Commission ALIAS openarm_commission)
target_compile_features(openarm_commission PUBLIC c_std_11 PRIVATE cxx_std_17)
target_include_directories(openarm_commission
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
set_target_properties(openarm_commission PROPERTIES
    CXX_EXTENSIONS OFF
    EXPORT_NAME Commission
    POSITION_INDEPENDENT_CODE ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(openarm_commission PRIVATE
        -Wall -Wextra -Wpedantic -Werror)
    if(OA_COMMISSION_ENABLE_SANITIZERS)
        target_compile_options(openarm_commission PUBLIC
            -fno-omit-frame-pointer -fsanitize=address,undefined)
        target_link_options(openarm_commission PUBLIC
            -fno-omit-frame-pointer -fsanitize=address,undefined)
    endif()
endif()

if(BUILD_TESTING)
    find_package(Threads REQUIRED)
    add_library(openarm_commission_test STATIC
        src/calibration_session.cpp
        src/c_api.cpp)
    target_compile_definitions(openarm_commission_test PRIVATE
        OPENARM_COMMISSION_ENABLE_TEST_HOOKS=1)
    target_compile_features(openarm_commission_test
        PUBLIC c_std_11 PRIVATE cxx_std_17)
    target_include_directories(openarm_commission_test
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
    set_target_properties(openarm_commission_test PROPERTIES
        CXX_EXTENSIONS OFF
        POSITION_INDEPENDENT_CODE ON)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(openarm_commission_test PRIVATE
            -Wall -Wextra -Wpedantic -Werror)
        if(OA_COMMISSION_ENABLE_SANITIZERS)
            target_compile_options(openarm_commission_test PUBLIC
                -fno-omit-frame-pointer -fsanitize=address,undefined)
            target_link_options(openarm_commission_test PUBLIC
                -fno-omit-frame-pointer -fsanitize=address,undefined)
        endif()
    endif()
    add_executable(openarm_commission_tests tests/test_commission.cpp)
    target_link_libraries(openarm_commission_tests PRIVATE
        openarm_commission_test Threads::Threads)
    target_include_directories(openarm_commission_tests PRIVATE src)
    target_compile_features(openarm_commission_tests PRIVATE cxx_std_17)
    set_target_properties(openarm_commission_tests PROPERTIES CXX_EXTENSIONS OFF)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(openarm_commission_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME openarm_commission_tests COMMAND openarm_commission_tests)

    add_executable(openarm_commission_c11_consumer tests/c11_consumer.c)
    target_link_libraries(openarm_commission_c11_consumer PRIVATE OpenArm::Commission)
    target_compile_features(openarm_commission_c11_consumer PRIVATE c_std_11)
    set_target_properties(openarm_commission_c11_consumer PROPERTIES LINKER_LANGUAGE CXX)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(openarm_commission_c11_consumer PRIVATE
            -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME openarm_commission_c11_consumer
             COMMAND openarm_commission_c11_consumer)
endif()

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_commissionConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/openarm_commissionConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_commissionConfig.cmake"
    INSTALL_DESTINATION lib/cmake/openarm_commission)

install(TARGETS openarm_commission
    EXPORT openarm_commissionTargets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include)
install(FILES include/openarm_commission.h DESTINATION include)
install(EXPORT openarm_commissionTargets
    FILE openarm_commissionTargets.cmake
    NAMESPACE OpenArm::
    DESTINATION lib/cmake/openarm_commission)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_commissionConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_commissionConfigVersion.cmake"
    DESTINATION lib/cmake/openarm_commission)
