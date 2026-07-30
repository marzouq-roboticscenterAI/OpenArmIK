set(install_prefix "${OA_RUNTIME_BUILD_DIR}/abi-v1-install")
set(consumer_build "${OA_RUNTIME_BUILD_DIR}/abi-v1-install-consumer")
set(build_jobs "$ENV{OPENARM_BUILD_JOBS}")
if(build_jobs STREQUAL "")
    set(build_jobs "1")
endif()
if(NOT build_jobs MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "OPENARM_BUILD_JOBS must be positive")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${OA_RUNTIME_BUILD_DIR}" --prefix "${install_prefix}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Runtime install failed: ${result}")
endif()
set(runtime_config
    "${install_prefix}/lib/cmake/openarm_runtime/openarm_runtimeConfig.cmake")
file(READ "${runtime_config}" runtime_config_content)
string(FIND "${runtime_config_content}"
    "set_and_check(openarm_runtime_V1_INCLUDE_DIR" include_capture_index)
string(FIND "${runtime_config_content}" "find_dependency(" dependency_index)
if(include_capture_index LESS 0 OR dependency_index LESS 0 OR
   NOT include_capture_index LESS dependency_index)
    message(FATAL_ERROR
        "Runtime V1 include must be captured before every dependency lookup")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}"
    -DOA_NM=${OA_NM}
    -DOA_RUNTIME_ARCHIVE=${install_prefix}/lib/libopenarm_runtime.a
    -DOA_EXPECTED_SYMBOLS=${OA_RUNTIME_SOURCE_DIR}/tests/abi_v1/expected_symbols.txt
    -P ${OA_RUNTIME_SOURCE_DIR}/tests/check_symbol_manifest.cmake
    RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Installed Runtime symbol manifest failed: ${result}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${OA_RUNTIME_SOURCE_DIR}/tests/install_consumer" -B "${consumer_build}" -DCMAKE_BUILD_TYPE=Release "-DCMAKE_PREFIX_PATH=${OA_DEPENDENCY_PREFIX};${install_prefix}" -Dopenarm_runtime_DIR=${install_prefix}/lib/cmake/openarm_runtime -DOA_EXPECTED_RUNTIME_V1_INCLUDE=${install_prefix}/include/openarm_runtime_abi_v1 RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Installed consumer configure failed: ${result}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${consumer_build}" --parallel "${build_jobs}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Installed consumer build failed: ${result}")
endif()
foreach(canary IN ITEMS runtime_v1_installed_c11 runtime_v1_installed_cxx17
        runtime_v1_installed_current_c11 runtime_v1_installed_current_cxx17
        runtime_units_installed_c11 runtime_units_installed_cxx17)
    execute_process(COMMAND "${consumer_build}/${canary}" RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${canary} failed: ${result}")
    endif()
endforeach()

set(mismatch_source "${OA_RUNTIME_BUILD_DIR}/abi-v1-version-mismatch-source")
set(mismatch_build "${OA_RUNTIME_BUILD_DIR}/abi-v1-version-mismatch-build")
file(MAKE_DIRECTORY "${mismatch_source}")
file(WRITE "${mismatch_source}/CMakeLists.txt"
    "cmake_minimum_required(VERSION 3.16)\n"
    "project(openarm_commission_exact_mismatch NONE)\n"
    "find_package(openarm_commission 0.1.1 EXACT CONFIG REQUIRED)\n")
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${mismatch_source}"
    -B "${mismatch_build}" "-DCMAKE_PREFIX_PATH=${OA_DEPENDENCY_PREFIX}"
    RESULT_VARIABLE mismatch_result OUTPUT_VARIABLE mismatch_output
    ERROR_VARIABLE mismatch_error)
if(mismatch_result EQUAL 0)
    message(FATAL_ERROR "Commission 0.1.1 unexpectedly satisfied exact 0.1.0 ABI dependency")
endif()
if(NOT "${mismatch_output}${mismatch_error}" MATCHES "0\\.1\\.0")
    message(FATAL_ERROR "Commission exact-version failure did not identify installed 0.1.0")
endif()
