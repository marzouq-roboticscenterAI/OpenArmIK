cmake_minimum_required(VERSION 3.16)
project(openarm_can_diagnostics VERSION 1.0.0 LANGUAGES C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(OA_CAN_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(CMAKE_C_EXTENSIONS OFF)

add_library(openarm_can STATIC
    src/openarm_can.c
    src/openarm_can_linux.c
)
add_library(OpenArm::Can ALIAS openarm_can)
set_target_properties(openarm_can PROPERTIES EXPORT_NAME Can)
target_include_directories(openarm_can PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(openarm_can PUBLIC c_std_11)
set_target_properties(openarm_can PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)

if(MSVC)
    target_compile_options(openarm_can PRIVATE /W4 /WX)
elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(openarm_can PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
        -Wshadow -Wformat=2 -Wundef -Wcast-align -Wstrict-prototypes
        -Wmissing-prototypes
    )
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        target_compile_options(openarm_can PRIVATE -Wcast-align=strict)
    endif()
    if(OA_CAN_ENABLE_SANITIZERS)
        target_compile_options(openarm_can PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(openarm_can PRIVATE -fsanitize=address,undefined)
    endif()
endif()

configure_package_config_file(
    cmake/OpenArmCanConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/OpenArmCanConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenArmCan)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/OpenArmCanConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
configure_package_config_file(
    cmake/openarm_canConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_canConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_can)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_canConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(TARGETS openarm_can EXPORT OpenArmCanTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES include/openarm_can.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT OpenArmCanTargets
    FILE OpenArmCanTargets.cmake
    NAMESPACE OpenArm::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenArmCan)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/OpenArmCanConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/OpenArmCanConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenArmCan)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_canConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/openarm_canConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/openarm_can)

include(CTest)
if(BUILD_TESTING)
    add_executable(openarm_can_tests tests/test_openarm_can.c)
    target_link_libraries(openarm_can_tests PRIVATE openarm_can)
    target_include_directories(openarm_can_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    if(NOT MSVC)
        target_link_libraries(openarm_can_tests PRIVATE m)
    endif()
    target_compile_features(openarm_can_tests PRIVATE c_std_11)
    set_target_properties(openarm_can_tests PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    if(MSVC)
        target_compile_options(openarm_can_tests PRIVATE /W4 /WX)
    elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(openarm_can_tests PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
            -Wshadow -Wformat=2 -Wundef -Wcast-align -Wstrict-prototypes
            -Wmissing-prototypes
        )
        if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
            target_compile_options(openarm_can_tests PRIVATE -Wcast-align=strict)
        endif()
        if(OA_CAN_ENABLE_SANITIZERS)
            target_compile_options(openarm_can_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
            target_link_options(openarm_can_tests PRIVATE -fsanitize=address,undefined)
        endif()
    endif()
    add_test(NAME openarm_can_tests COMMAND openarm_can_tests)
endif()
