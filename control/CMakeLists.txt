cmake_minimum_required(VERSION 3.16)
project(openarm_control VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)

option(OA_CONTROL_BUILD_TESTS "Build the compiled control tests" ON)
option(OA_CONTROL_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

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

install(TARGETS openarm_control
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES include/openarm_control.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include(CTest)
if(BUILD_TESTING AND OA_CONTROL_BUILD_TESTS)
    add_executable(openarm_control_tests tests/test_control.cpp)
    target_link_libraries(openarm_control_tests PRIVATE openarm_control)
    target_compile_options(openarm_control_tests PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2)
    add_test(NAME openarm_control_tests COMMAND openarm_control_tests)

    add_executable(openarm_control_c11_abi tests/c11_abi_consumer.c)
    target_link_libraries(openarm_control_c11_abi PRIVATE openarm_control)
    set_target_properties(openarm_control_c11_abi PROPERTIES LINKER_LANGUAGE CXX)
    target_compile_options(openarm_control_c11_abi PRIVATE
        -Wall -Wextra -Wpedantic -Werror -Wshadow -Wformat=2)
    add_test(NAME openarm_control_c11_abi COMMAND openarm_control_c11_abi)
endif()
