cmake_minimum_required(VERSION 3.16)
project(openarm_model LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(OA_MODEL_BUILD_TESTS "Build model tests" ON)
option(OA_MODEL_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(OA_MODEL_COVERAGE "Enable gcov coverage instrumentation" OFF)
set(OA_DESCRIPTION_ROOT "" CACHE PATH "Pinned openarm_description checkout for regeneration")
set(OA_XACRO_EXECUTABLE "" CACHE FILEPATH "xacro executable for regeneration")
set(OA_XACRO_PYTHONPATH "" CACHE STRING "PYTHONPATH required by xacro")
set(OA_XACRO_AMENT_PREFIX "" CACHE STRING "Base AMENT_PREFIX_PATH required by xacro")

add_library(openarm_model STATIC src/openarm_model.c)
target_include_directories(openarm_model PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(openarm_model PUBLIC c_std_11)
set_target_properties(openarm_model PROPERTIES C_EXTENSIONS OFF)
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
  add_executable(oa_model_abi_v1_canary tests/abi_v1_canary.c)
  target_link_libraries(oa_model_abi_v1_canary PRIVATE openarm_model)
  target_compile_options(oa_model_abi_v1_canary PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME oa_model_abi_v1_canary COMMAND oa_model_abi_v1_canary)
  add_executable(oa_model_reference_driver tests/reference_driver.c)
  target_link_libraries(oa_model_reference_driver PRIVATE openarm_model)
  target_compile_options(oa_model_reference_driver PRIVATE -Wall -Wextra -Wpedantic -Werror)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME oa_model_python_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_reference.py
      ${CMAKE_CURRENT_SOURCE_DIR}/generated/openarm_v10_bimanual.urdf
      $<TARGET_FILE:oa_model_reference_driver>)
  if(OA_DESCRIPTION_ROOT AND OA_XACRO_EXECUTABLE AND OA_XACRO_PYTHONPATH)
    add_test(NAME oa_model_generator_determinism
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_generator.py
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_model.py
        ${OA_DESCRIPTION_ROOT}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/generated/oa_model_data.inc
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/openarm_v10_bimanual.urdf
        ${OA_XACRO_EXECUTABLE} ${OA_XACRO_PYTHONPATH} ${OA_XACRO_AMENT_PREFIX})
  else()
    message(STATUS "Model regeneration test disabled; set OA_DESCRIPTION_ROOT, OA_XACRO_EXECUTABLE, and OA_XACRO_PYTHONPATH")
  endif()
endif()
