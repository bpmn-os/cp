# CP interface test (always built - header-only)
add_executable(test_cp cp.cpp)
target_link_libraries(test_cp PRIVATE cp)
add_test(NAME test_cp COMMAND test_cp)

# SCIP tests (only if SCIP was built)
if(TARGET libscip)
    add_subdirectory(scip)
endif()
