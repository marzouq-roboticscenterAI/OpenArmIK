cmake_minimum_required(VERSION 3.16)
project(openarm_model LANGUAGES C)

option(OA_MODEL_BUILD_TESTS "Build model tests" ON)
option(OA_MODEL_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(OA_MODEL_COVERAGE "Enable gcov coverage instrumentation" OFF)
set(OA_DESCRIPTION_ROOT "/home/signalprocessing-dev/OpenArmIK/upstream/openarm_description" CACHE PATH "Pinned openarm_description checkout used by generator tests")

add_library(openarm_model STATIC src/openarm_model.c)
target_include_directories(openarm_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(openarm_model PUBLIC c_std_11)
target_compile_options(openarm_model PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(openarm_model PUBLIC m)
if(OA_MODEL_SANITIZERS)
  target_compile_options(openarm_model PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(openarm_model INTERFACE -fsanitize=address,undefined)
endif()
if(OA_MODEL_COVERAGE)
  target_compile_options(openarm_model PRIVATE --coverage -O0)
  target_link_options(openarm_model INTERFACE --coverage)
endif()

if(OA_MODEL_BUILD_TESTS)
  enable_testing()
  add_executable(oa_model_tests tests/test_model.c)
  target_link_libraries(oa_model_tests PRIVATE openarm_model)
  target_compile_options(oa_model_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME oa_model_tests COMMAND oa_model_tests)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME oa_model_generator_determinism
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_generator.py
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_model.py
      ${OA_DESCRIPTION_ROOT}
      ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/oa_model_data.inc)
endif()
