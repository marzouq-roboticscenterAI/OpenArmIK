set(prefix "${OA_BUILD_DIR}/install-consumer-prefix")
set(consumer_build "${OA_BUILD_DIR}/install-consumer-build")
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
        "-DCMAKE_PREFIX_PATH=${prefix}"
    RESULT_VARIABLE configure_result)
if(NOT configure_result EQUAL 0)
    message(FATAL_ERROR "installed consumer configure failed: ${configure_result}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${consumer_build}" --parallel
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
