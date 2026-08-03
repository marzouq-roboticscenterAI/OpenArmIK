execute_process(COMMAND "${OA_NM}" -g --defined-only --format=posix "${OA_RUNTIME_ARCHIVE}"
    RESULT_VARIABLE nm_result OUTPUT_VARIABLE nm_output ERROR_VARIABLE nm_error)
if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "nm failed: ${nm_error}")
endif()
string(REPLACE "\n" ";" nm_lines "${nm_output}")
set(actual_symbols)
foreach(line IN LISTS nm_lines)
    if(line MATCHES "^(oa_runtime_[a-z0-9_]+) T ")
        list(APPEND actual_symbols "${CMAKE_MATCH_1}")
    endif()
endforeach()
list(REMOVE_DUPLICATES actual_symbols)
list(SORT actual_symbols)
file(STRINGS "${OA_EXPECTED_SYMBOLS}" expected_symbols)
list(SORT expected_symbols)
list(LENGTH actual_symbols actual_count)
list(LENGTH expected_symbols expected_count)
# 50 frozen V1 entry points plus the 7 additive entry points declared in
# openarm_runtime_motion.h. V1 itself is unchanged: nothing was removed, no
# struct layout moved, and openarm_runtime.h remains byte-identical to its
# frozen copy. Raising this number requires the same explicit review as any
# other freeze update.
set(OA_EXPECTED_SYMBOL_COUNT 57)
if(NOT actual_count EQUAL OA_EXPECTED_SYMBOL_COUNT OR
   NOT expected_count EQUAL OA_EXPECTED_SYMBOL_COUNT)
    message(FATAL_ERROR "Runtime symbol count mismatch: archive=${actual_count}, expected=${expected_count}, required=${OA_EXPECTED_SYMBOL_COUNT}")
endif()
if(NOT actual_symbols STREQUAL expected_symbols)
    message(FATAL_ERROR "Runtime V1 symbol manifest mismatch")
endif()
