#include "cp.h"
#include <cassert>
#include <iostream>

#define GREEN "\033[32m"
#define RESET "\033[0m"

// Test 1: collection() wrapper creates Expression with Operator::collection
void test_collection_wrapper() {
  CP::Model model;
  auto& key = model.addIntegerVariable("key");

  // Create collection expression
  CP::Expression collExpr = CP::collection(key);

  // Verify it has the right operator
  assert(collExpr._operator == CP::Expression::Operator::collection);
  assert(collExpr.operands.size() == 1);

  // Verify the operand is the key variable
  assert(std::holds_alternative<std::reference_wrapper<const CP::Variable>>(collExpr.operands[0]));
  const CP::Variable& varRef = std::get<std::reference_wrapper<const CP::Variable>>(collExpr.operands[0]);
  assert(&varRef == &key);

  std::cout << GREEN << "Test 1 PASSED: collection() wrapper creates correct Expression" << RESET << std::endl;
}

// Test 2: count() wrapper creates custom operator expression
void test_count_wrapper() {
  CP::Model model;
  auto& key = model.addIntegerVariable("key");

  // Create count(collection(key)) expression
  CP::Expression countExpr = CP::count(CP::collection(key));

  // Verify it's a custom operator
  assert(countExpr._operator == CP::Expression::Operator::custom);
  assert(countExpr.operands.size() == 2);  // custom index + collection expression

  // First operand should be the custom operator index for "count"
  assert(std::holds_alternative<size_t>(countExpr.operands[0]));

  // Second operand should be the collection expression
  assert(std::holds_alternative<CP::Expression>(countExpr.operands[1]));
  const CP::Expression& collExpr = std::get<CP::Expression>(countExpr.operands[1]);
  assert(collExpr._operator == CP::Expression::Operator::collection);

  std::cout << GREEN << "Test 2 PASSED: count(collection(key)) creates correct Expression" << RESET << std::endl;
}

int main() {
  int testNum = 0;

  try {
    test_collection_wrapper();
    testNum++;

    test_count_wrapper();
    testNum++;
  } catch (const std::exception& e) {
    std::cerr << "Test " << (testNum + 1) << " FAILED: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\n" << GREEN << "All " << testNum << " collection wrapper tests PASSED" << RESET << std::endl;
  return 0;
}
