# SCIP Adapter Tests
add_executable(test_scip test.cpp)
target_link_libraries(test_scip PRIVATE cp libscip cp-scip)
add_test(NAME test_scip COMMAND test_scip)

# SCIP Collection Operations Tests
add_executable(test_scip_collection test_collection.cpp)
target_link_libraries(test_scip_collection PRIVATE cp libscip cp-scip)
add_test(NAME test_scip_collection COMMAND test_scip_collection)
