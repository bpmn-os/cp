add_executable(test_hexaly test.cpp)
target_link_libraries(test_hexaly PRIVATE cp cp-hexaly)
add_test(NAME test_hexaly COMMAND test_hexaly)

add_executable(test_hexaly_collection test_collection.cpp)
target_link_libraries(test_hexaly_collection PRIVATE cp cp-hexaly)
add_test(NAME test_hexaly_collection COMMAND test_hexaly_collection)
