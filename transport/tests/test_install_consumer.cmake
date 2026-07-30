set(prefix "${OA_BUILD_DIR}/install-consumer-prefix")
set(consumer_build "${OA_BUILD_DIR}/install-consumer-build")
set(openarm_build_jobs "$ENV{OPENARM_BUILD_JOBS}")
if(openarm_build_jobs STREQUAL "")
    set(openarm_build_jobs "2")
endif()
if(NOT openarm_build_jobs MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "OPENARM_BUILD_JOBS must be a positive integer: ${openarm_build_jobs}")
endif()
file(REMOVE_RECURSE "${prefix}" "${consumer_build}")

execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${OA_BUILD_DIR}" --prefix "${prefix}"
    RESULT_VARIABLE install_result)
if(NOT install_result EQUAL 0)
    message(FATAL_ERROR "transport install failed: ${install_result}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}"
        -S "${OA_SOURCE_DIR}/tests/install_consumer"
        -B "${consumer_build}"
        "-DCMAKE_PREFIX_PATH=${prefix};${OA_CAN_PREFIX}"
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
    COMMAND "${consumer_build}/openarm_transport_installed_consumer"
    RESULT_VARIABLE run_result)
if(NOT run_result EQUAL 0)
    message(FATAL_ERROR "installed consumer run failed: ${run_result}")
endif()
