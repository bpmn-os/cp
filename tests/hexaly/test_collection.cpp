#include "cp.h"
#include "hexaly/hexaly_adapter.h"
#include <cassert>
#include <iostream>
#include <cmath>

#define GREEN "\033[32m"
#define RESET "\033[0m"

// Test 1: count(collection(key)) with variable key
void test_count_collection_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0};
    if (k == 2) return std::vector<double>{60.0, 70.0, 80.0, 90.0};
    return std::unexpected("Collection key " + std::to_string(k) + " not found");
  }, 3);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 2.0);
  auto& result = model.addVariable(CP::Variable::Type::INTEGER, "result", 0.0, 10.0);

  model.addConstraint(result == CP::count(CP::Collection(key)));
  model.addConstraint(key == 0.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());

  double keyVal = solution->getVariableValue(key).value();
  double resultVal = solution->getVariableValue(result).value();

  assert(std::abs(keyVal - 0.0) < 1e-5);
  assert(std::abs(resultVal - 3.0) < 1e-5);

  std::cout << GREEN << "Test 1 PASSED: count(collection(key)) with key=0 returns 3" << RESET << std::endl;
}

// Test 2: sum(collection(key))
void test_sum_collection_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{5.0, 15.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::sum(CP::Collection(key)));
  model.addConstraint(key == 1.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);

  std::cout << GREEN << "Test 2 PASSED: sum(collection(key)) with key=1 returns 20" << RESET << std::endl;
}

// Test 3: avg(collection(key))
void test_avg_collection_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 50.0);

  model.addConstraint(result == CP::avg(CP::Collection(key)));

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);

  std::cout << GREEN << "Test 3 PASSED: avg(collection(key)) with key=0 returns 20" << RESET << std::endl;
}

// Test 4: max(collection(key))
void test_max_collection_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 50.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::max(CP::Collection(key)));

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 50.0) < 1e-5);

  std::cout << GREEN << "Test 4 PASSED: max(collection(key)) with key=0 returns 50" << RESET << std::endl;
}

// Test 5: min(collection(key))
void test_min_collection_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{30.0, 10.0, 50.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::min(CP::Collection(key)));

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 10.0) < 1e-5);

  std::cout << GREEN << "Test 5 PASSED: min(collection(key)) with key=0 returns 10" << RESET << std::endl;
}

// Test 6: element_of(constant, collection(key))
void test_element_of_constant_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::element_of(20.0, CP::Collection(key)));
  model.addConstraint(key == 0.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);

  std::cout << GREEN << "Test 6 PASSED: element_of(20, collection(0)) returns 1" << RESET << std::endl;
}

// Test 7: element_of when value NOT in collection
void test_element_of_not_found_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::element_of(25.0, CP::Collection(key)));

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 0.0) < 1e-5);

  std::cout << GREEN << "Test 7 PASSED: element_of(25, collection(0)) returns 0" << RESET << std::endl;
}

// Test 8: not_element_of
void test_not_element_of_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::not_element_of(25.0, CP::Collection(key)));

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);

  std::cout << GREEN << "Test 8 PASSED: not_element_of(25, collection(0)) returns 1" << RESET << std::endl;
}

// Test 9: Collection(key)[constant_index]
void test_at_constant_index_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::Collection(key)[2.0]);
  model.addConstraint(key == 0.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);

  std::cout << GREEN << "Test 9 PASSED: Collection(0)[2] returns 20" << RESET << std::endl;
}

// Test 10: count(Collection(constant_key)) - constant key
void test_count_constant_key_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 10.0);

  model.addConstraint(result == CP::count(CP::Collection(0.0)));

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 3.0) < 1e-5);

  std::cout << GREEN << "Test 10 PASSED: count(Collection(0.0)) with constant key returns 3" << RESET << std::endl;
}

// Test 11: at() with different collection keys
void test_at_different_keys_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0, 60.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::Collection(key)[2.0]);
  model.addConstraint(key == 0.0);

  CP::HexalySolver solver1(model);
  auto solution1 = solver1.solve(model, 5.0);

  assert(solution1.has_value());
  double resultVal1 = solution1->getVariableValue(result).value();
  assert(std::abs(resultVal1 - 20.0) < 1e-5);

  // Now test with key=1
  CP::Model model2;
  model2.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0, 60.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key2 = model2.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& result2 = model2.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model2.addConstraint(result2 == CP::Collection(key2)[2.0]);
  model2.addConstraint(key2 == 1.0);

  CP::HexalySolver solver2(model2);
  auto solution2 = solver2.solve(model2, 5.0);

  assert(solution2.has_value());
  double resultVal2 = solution2->getVariableValue(result2).value();
  assert(std::abs(resultVal2 - 50.0) < 1e-5);

  std::cout << GREEN << "Test 11 PASSED: at(2, collection(key)) returns correct values for different keys" << RESET << std::endl;
}

// Test 12: element_of(variable_value, collection(key))
void test_element_of_variable_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::element_of(value, CP::Collection(key)));
  model.addConstraint(value == 20.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);

  std::cout << GREEN << "Test 12 PASSED: element_of(variable, collection(0)) with value=20 returns 1" << RESET << std::endl;
}

// Test 13: element_of(variable_value, collection(key)) - value NOT in collection
void test_element_of_variable_not_found_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::element_of(value, CP::Collection(key)));
  model.addConstraint(value == 25.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 0.0) < 1e-5);

  std::cout << GREEN << "Test 13 PASSED: element_of(variable, collection(0)) with value=25 returns 0" << RESET << std::endl;
}

// Test 14: element_of(variable_value, collection(variable_key))
void test_element_of_variable_both_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0, 60.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 60.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::element_of(value, CP::Collection(key)));
  model.addConstraint(key == 1.0);
  model.addConstraint(value == 50.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);

  std::cout << GREEN << "Test 14 PASSED: element_of(variable, collection(variable_key)) returns 1" << RESET << std::endl;
}

// Test 15: not_element_of(variable_value, collection(key)) - value IS in collection
void test_not_element_of_variable_found_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::not_element_of(value, CP::Collection(key)));
  model.addConstraint(value == 20.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 0.0) < 1e-5);

  std::cout << GREEN << "Test 15 PASSED: not_element_of(variable, collection(0)) with value=20 returns 0" << RESET << std::endl;
}

// Test 16: not_element_of(variable_value, collection(key)) - value NOT in collection
void test_not_element_of_variable_not_found_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  model.addConstraint(result == CP::not_element_of(value, CP::Collection(key)));
  model.addConstraint(value == 35.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);

  std::cout << GREEN << "Test 16 PASSED: not_element_of(variable, collection(0)) with value=35 returns 1" << RESET << std::endl;
}

// Test 17: at(variable_index, collection(key))
void test_at_variable_index_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
  auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 1.0, 3.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::Collection(key)[index]);
  model.addConstraint(index == 2.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);

  std::cout << GREEN << "Test 17 PASSED: at(variable_index, collection(0)) with index=2 returns 20" << RESET << std::endl;
}

// Test 18: at(variable_index, collection(variable_key))
void test_at_variable_both_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 1) return std::vector<double>{40.0, 50.0, 60.0};
    return std::unexpected("Collection key not found");
  }, 2);

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
  auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 1.0, 3.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::Collection(key)[index]);
  model.addConstraint(key == 1.0);
  model.addConstraint(index == 3.0);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 60.0) < 1e-5);

  std::cout << GREEN << "Test 18 PASSED: at(variable_index, collection(variable_key)) returns 60" << RESET << std::endl;
}

// Test 19: Collection(constant_key)[constant_index] - both constant
void test_at_both_constant_hexaly() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 0) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  }, 1);

  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::Collection(0.0)[2.0]);

  CP::HexalySolver solver(model);
  auto solution = solver.solve(model, 5.0);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);

  std::cout << GREEN << "Test 19 PASSED: Collection(0.0)[2.0] with both constant returns 20" << RESET << std::endl;
}

int main() {
  int testNum = 0;

  try {
    test_count_collection_hexaly();
    testNum++;

    test_sum_collection_hexaly();
    testNum++;

    test_avg_collection_hexaly();
    testNum++;

    test_max_collection_hexaly();
    testNum++;

    test_min_collection_hexaly();
    testNum++;

    test_element_of_constant_hexaly();
    testNum++;

    test_element_of_not_found_hexaly();
    testNum++;

    test_not_element_of_hexaly();
    testNum++;

    test_at_constant_index_hexaly();
    testNum++;

    test_count_constant_key_hexaly();
    testNum++;

    test_at_different_keys_hexaly();
    testNum++;

    test_element_of_variable_hexaly();
    testNum++;

    test_element_of_variable_not_found_hexaly();
    testNum++;

    test_element_of_variable_both_hexaly();
    testNum++;

    test_not_element_of_variable_found_hexaly();
    testNum++;

    test_not_element_of_variable_not_found_hexaly();
    testNum++;

    test_at_variable_index_hexaly();
    testNum++;

    test_at_variable_both_hexaly();
    testNum++;

    test_at_both_constant_hexaly();
    testNum++;
  }
  catch (const std::exception& e) {
    std::cerr << "Test " << (testNum + 1) << " FAILED: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\n" << GREEN << "All " << testNum << " Hexaly collection tests PASSED" << RESET << std::endl;
  return 0;
}
