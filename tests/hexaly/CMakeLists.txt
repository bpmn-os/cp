add_executable(test_hexaly test.cpp)
target_link_libraries(test_hexaly PRIVATE cp cp-hexaly)
add_test(NAME test_hexaly COMMAND test_hexaly)
