cmake_minimum_required(VERSION 3.16)
project(openarm_can_diagnostics VERSION 1.0.0 LANGUAGES C)

option(OA_CAN_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
set(CMAKE_C_EXTENSIONS OFF)

add_library(openarm_can STATIC
    src/openarm_can.c
    src/openarm_can_linux.c
)
target_include_directories(openarm_can PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
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
