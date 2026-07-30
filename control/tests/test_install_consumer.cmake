set(install_prefix "${OA_CONTROL_BUILD_DIR}/public-header-install")
set(consumer_build "${OA_CONTROL_BUILD_DIR}/public-header-install-consumer")
set(openarm_build_jobs "$ENV{OPENARM_BUILD_JOBS}")
if(openarm_build_jobs STREQUAL "")
    set(openarm_build_jobs "2")
endif()
if(NOT openarm_build_jobs MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "OPENARM_BUILD_JOBS must be a positive integer: ${openarm_build_jobs}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${OA_CONTROL_BUILD_DIR}"
            --prefix "${install_prefix}"
    RESULT_VARIABLE control_install_result)
if(NOT control_install_result EQUAL 0)
    message(FATAL_ERROR "control install failed: ${control_install_result}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}"
            -S "${OA_SOURCE_DIR}/tests/install_consumer"
            -B "${consumer_build}"
            -DCMAKE_BUILD_TYPE=Release
            "-DCMAKE_PREFIX_PATH=${install_prefix};${OA_MODEL_PREFIX}"
            -Dopenarm_control_DIR=${install_prefix}/lib/cmake/openarm_control
            -Dopenarm_model_DIR=${OA_MODEL_PREFIX}/lib/cmake/openarm_model
    RESULT_VARIABLE configure_result)
if(NOT configure_result EQUAL 0)
    message(FATAL_ERROR "installed consumer configure failed: ${configure_result}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${consumer_build}" --parallel "${openarm_build_jobs}"
    RESULT_VARIABLE build_result)
if(NOT build_result EQUAL 0)
    message(FATAL_ERROR "installed consumer build failed: ${build_result}")
endif()

execute_process(
    COMMAND "${consumer_build}/installed_headers_c11"
    RESULT_VARIABLE c11_result)
if(NOT c11_result EQUAL 0)
    message(FATAL_ERROR "installed C11 consumer failed: ${c11_result}")
endif()

execute_process(
    COMMAND "${consumer_build}/installed_headers_cxx17"
    RESULT_VARIABLE cxx17_result)
if(NOT cxx17_result EQUAL 0)
    message(FATAL_ERROR "installed C++17 consumer failed: ${cxx17_result}")
endif()
