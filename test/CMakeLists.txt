include (GoogleTest)
include_directories(${gtest_SOURCE_DIR}/include)

set(tests
    sysstats_TEST.cpp)

foreach (GTEST_SOURCE_file ${tests})
    string(REGEX REPLACE ".cpp" "" BINARY_NAME ${GTEST_SOURCE_file})
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file})

    add_dependencies(${BINARY_NAME}
        sys_stats
        gtest
        gtest_main)

    target_link_libraries(${BINARY_NAME}
        gtest_main
        gtest
        sys_stats)

   add_test(NAME    ${BINARY_NAME}
            COMMAND ${BINARY_NAME}
                    --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
   set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)
endforeach()
