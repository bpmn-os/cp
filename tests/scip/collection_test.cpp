#include "cp.h"
#include "scip/scip_adapter.h"
#include <cassert>
#include <iostream>
#include <cmath>

#define GREEN "\033[32m"
#define RESET "\033[0m"

// Test 1: count(collection(key)) with variable key - SCIP solving
void test_count_collection_scip() {
  CP::Model model;

  // Set up collections
  // collection(1) = [10, 20, 30] -> count = 3
  // collection(2) = [40, 50] -> count = 2
  // collection(3) = [60, 70, 80, 90] -> count = 4
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 2) return std::vector<double>{40.0, 50.0};
    if (k == 3) return std::vector<double>{60.0, 70.0, 80.0, 90.0};
    return std::unexpected("Collection key " + std::to_string(k) + " not found");
  });

  // Create variables
  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 3.0);
  auto& result = model.addVariable(CP::Variable::Type::INTEGER, "result", 0.0, 10.0);

  // Constraint: result = count(collection(key))
  model.addConstraint(result == CP::count(CP::collection(key)));

  // Force key to be 1
  model.addConstraint(key == 1.0);

  // Solve with SCIP
  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  assert(solution->getStatus() == CP::Solution::Status::OPTIMAL);

  // Verify result
  double keyVal = solution->getVariableValue(key).value();
  double resultVal = solution->getVariableValue(result).value();

  assert(std::abs(keyVal - 1.0) < 1e-5);
  assert(std::abs(resultVal - 3.0) < 1e-5);  // count([10, 20, 30]) = 3

  std::cout << GREEN << "Test 1 PASSED: count(collection(key)) with key=1 returns 3" << RESET << std::endl;
}

// Test 2: sum(collection(key))
void test_sum_collection_scip() {
  CP::Model model;

  // collection(1) = [10, 20, 30] -> sum = 60
  // collection(2) = [5, 15] -> sum = 20
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 2) return std::vector<double>{5.0, 15.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 2.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::sum(CP::collection(key)));
  model.addConstraint(key == 2.0);  // Force key to be 2

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);  // sum([5, 15]) = 20

  std::cout << GREEN << "Test 2 PASSED: sum(collection(key)) with key=2 returns 20" << RESET << std::endl;
}

// Test 3: avg(collection(key))
void test_avg_collection_scip() {
  CP::Model model;

  // collection(1) = [10, 20, 30] -> avg = 20
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 50.0);

  model.addConstraint(result == CP::avg(CP::collection(key)));

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);  // avg([10, 20, 30]) = 20

  std::cout << GREEN << "Test 3 PASSED: avg(collection(key)) with key=1 returns 20" << RESET << std::endl;
}

// Test 4: max(collection(key))
void test_max_collection_scip() {
  CP::Model model;

  // collection(1) = [10, 50, 30] -> max = 50
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 50.0, 30.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::max(CP::collection(key)));

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 50.0) < 1e-5);  // max([10, 50, 30]) = 50

  std::cout << GREEN << "Test 4 PASSED: max(collection(key)) with key=1 returns 50" << RESET << std::endl;
}

// Test 5: min(collection(key))
void test_min_collection_scip() {
  CP::Model model;

  // collection(1) = [30, 10, 50] -> min = 10
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{30.0, 10.0, 50.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model.addConstraint(result == CP::min(CP::collection(key)));

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 10.0) < 1e-5);  // min([30, 10, 50]) = 10

  std::cout << GREEN << "Test 5 PASSED: min(collection(key)) with key=1 returns 10" << RESET << std::endl;
}

// Test 6: element_of(constant, collection(key))
void test_element_of_constant_scip() {
  CP::Model model;

  // collection(1) = [10, 20, 30]
  // collection(2) = [40, 50]
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 2) return std::vector<double>{40.0, 50.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 2.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  // Check if 20.0 is in collection(key)
  model.addConstraint(result == CP::element_of(20.0, CP::collection(key)));
  model.addConstraint(key == 1.0);  // Force key to be 1

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);  // 20 is in [10, 20, 30]

  std::cout << GREEN << "Test 6 PASSED: element_of(20, collection(1)) returns 1" << RESET << std::endl;
}

// Test 7: element_of when value NOT in collection
void test_element_of_not_found_scip() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  // Check if 25.0 is in collection(key) - should be 0
  model.addConstraint(result == CP::element_of(25.0, CP::collection(key)));

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 0.0) < 1e-5);  // 25 is NOT in [10, 20, 30]

  std::cout << GREEN << "Test 7 PASSED: element_of(25, collection(1)) returns 0" << RESET << std::endl;
}

// Test 8: not_element_of
void test_not_element_of_scip() {
  CP::Model model;

  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 1.0);
  auto& result = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);

  // Check if 25.0 is NOT in collection(key) - should be 1
  model.addConstraint(result == CP::not_element_of(25.0, CP::collection(key)));

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 1.0) < 1e-5);  // 25 is NOT in [10, 20, 30]

  std::cout << GREEN << "Test 8 PASSED: not_element_of(25, collection(1)) returns 1" << RESET << std::endl;
}

// Test 9: at(constant_index, collection(key))
void test_at_constant_index_scip() {
  CP::Model model;

  // collection(1) = [10, 20, 30]
  // collection(2) = [40, 50]
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 2) return std::vector<double>{40.0, 50.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 2.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  // Get element at index 2 (1-based indexing)
  model.addConstraint(result == CP::at(2.0, CP::collection(key)));
  model.addConstraint(key == 1.0);  // Force key to be 1

  CP::SCIPSolver solver(model);
  auto solution = solver.solve(model);

  assert(solution.has_value());
  double resultVal = solution->getVariableValue(result).value();
  assert(std::abs(resultVal - 20.0) < 1e-5);  // at(2, [10, 20, 30]) = 20 (1-based)

  std::cout << GREEN << "Test 9 PASSED: at(2, collection(1)) returns 20" << RESET << std::endl;
}

// Test 10: at() with different collection keys
void test_at_different_keys_scip() {
  CP::Model model;

  // collection(1) = [10, 20, 30]
  // collection(2) = [40, 50, 60]
  model.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 2) return std::vector<double>{40.0, 50.0, 60.0};
    return std::unexpected("Collection key not found");
  });

  auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 2.0);
  auto& result = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  // Get element at index 2 from collection(key)
  model.addConstraint(result == CP::at(2.0, CP::collection(key)));

  // Test with key=1: at(2, [10,20,30]) should be 20
  model.addConstraint(key == 1.0);

  CP::SCIPSolver solver1(model);
  auto solution1 = solver1.solve(model);

  assert(solution1.has_value());
  double resultVal1 = solution1->getVariableValue(result).value();
  assert(std::abs(resultVal1 - 20.0) < 1e-5);

  // Now test with key=2: at(2, [40,50,60]) should be 50
  CP::Model model2;
  model2.setCollectionLookup([](double key) -> std::expected<std::vector<double>, std::string> {
    int k = (int)std::round(key);
    if (k == 1) return std::vector<double>{10.0, 20.0, 30.0};
    if (k == 2) return std::vector<double>{40.0, 50.0, 60.0};
    return std::unexpected("Collection key not found");
  });

  auto& key2 = model2.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 2.0);
  auto& result2 = model2.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);

  model2.addConstraint(result2 == CP::at(2.0, CP::collection(key2)));
  model2.addConstraint(key2 == 2.0);

  CP::SCIPSolver solver2(model2);
  auto solution2 = solver2.solve(model2);

  assert(solution2.has_value());
  double resultVal2 = solution2->getVariableValue(result2).value();
  assert(std::abs(resultVal2 - 50.0) < 1e-5);

  std::cout << GREEN << "Test 10 PASSED: at(2, collection(key)) returns correct values for different keys" << RESET << std::endl;
}

int main() {
  int testNum = 0;

  try {
    test_count_collection_scip();
    testNum++;

    test_sum_collection_scip();
    testNum++;

    test_avg_collection_scip();
    testNum++;

    test_max_collection_scip();
    testNum++;

    test_min_collection_scip();
    testNum++;

    test_element_of_constant_scip();
    testNum++;

    test_element_of_not_found_scip();
    testNum++;

    test_not_element_of_scip();
    testNum++;

    test_at_constant_index_scip();
    testNum++;

    test_at_different_keys_scip();
    testNum++;
  } catch (const std::exception& e) {
    std::cerr << "Test " << (testNum + 1) << " FAILED: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\n" << GREEN << "All " << testNum << " SCIP collection tests PASSED" << RESET << std::endl;
  return 0;
}
