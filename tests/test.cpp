#include <iostream>
#include <cassert>

#include "cp.h"

#define GREEN "\033[32m"
#define RESET "\033[0m" 

int main()
{
//  std::cout << ("Create model.") << std::endl;
  {
    CP::Model model;
 
    auto& x = model.addRealVariable("x");
    assert( x.stringify() == "x ∈ [ -infinity, infinity ]");
    auto& y = model.addBinaryVariable("y");
    assert( y.stringify() == "y ∈ { false, true }");
    auto& z = model.addIntegerVariable("z");
    assert( z.stringify() == "z ∈ { -infinity, ..., infinity }");
  
//std::cout << (x * 3 + z * 5).stringify() << std::endl;  
    assert( (x * 3 + z * 5).stringify() == "( x * 3.00 ) + ( z * 5.00 )");
    assert( (3 * x + 5 * z - 4).stringify() == "( ( 3.00 * x ) + ( 5.00 * z ) ) - 4.00");
    assert( (4 + 3 * x + z / 5).stringify() == "( 4.00 + ( 3.00 * x ) ) + ( z / 5.00 )");
    assert( (4 + 2 + 3*3 * x + z / 5 * 5).stringify() == "( 6.00 + ( 9.00 * x ) ) + ( ( z / 5.00 ) * 5.00 )");

    assert( (!y && y).stringify() == "( !y ) && y");
    assert( (y || !y).stringify() == "y || ( !y )");
    assert( ( !!y ).stringify() == "y");
    assert( ( !(!y) ).stringify() == "y");
  
    assert( CP::max( 0.0, x, 3 * z ).stringify() == "max( 0.00, x, 3.00 * z )");
    assert( CP::min( 0, x, 3 * z ).stringify() == "min( 0.00, x, 3.00 * z )");
  
    std::vector<CP::Expression> terms = { 0.0, x, 3 * z };
    assert( CP::max( terms ).stringify() == "max( 0.00, x, 3.00 * z )");
    assert( CP::min( terms ).stringify() == "min( 0.00, x, 3.00 * z )");

    assert( CP::if_then_else( y, x, 3 * z ).stringify() == "if_then_else( y, x, 3.00 * z )");
    auto& r = model.addVariable(CP::Variable::Type::BOOLEAN, "r", CP::if_then_else( y, x, 3 * z ) );

    assert( CP::n_ary_if( {{y, x}, {!y, 5}}, 3 * z ).stringify() == "n_ary_if( y, x, !y, 5.00, 3.00 * z )");
    auto& v = model.addVariable(CP::Variable::Type::INTEGER, "v", r + CP::n_ary_if( { {y, x}, {!y, 5} }, 3 * z ) );
//std::cout << v.stringify() << std::endl;  
    assert( v.stringify() == "v := r + n_ary_if( y, x, !y, 5.00, 3.00 * z )");

    auto& q = model.addVariable(CP::Variable::Type::BOOLEAN, "q", (x < z) );
    assert( q.stringify() == "q := x < z");
    auto& u = model.addVariable(CP::Variable::Type::BOOLEAN, "u", !(y && !y) );
    assert( u.stringify() == "u := !( y && ( !y ) )");

    auto& w = model.addVariable(CP::Variable::Type::BOOLEAN, "w", (y || !y) && !(y && !y) );
    assert( w.stringify() == "w := ( y || ( !y ) ) && ( !( y && ( !y ) ) )");


    auto& s = model.addSequence("s", 3 );
    assert( s.variables.size() == 3);
    assert( s.variables[0].name == "s[0]");
    assert( s.variables[1].name == "s[1]");
    assert( s.variables[2].name == "s[2]");
    assert( s.stringify() == "( s[0], s[1], s[2] ) is permutation of { 1, ..., 3 }");
 
    auto& a = model.addIndexedVariables(CP::Variable::Type::INTEGER, "a");
    model.addIndexedVariable(a, 0, 5);
    model.addIndexedVariable(a, w + 4);
    model.addIndexedVariable(a, a[1] + 5);
    assert( model.getIndexedVariables().back().stringify() == "a := { a[0] ∈ { 0, ..., 5 }, a[1] := w + 4.00, a[2] := a[1] + 5.00 }" );
    assert( a[1].stringify() == "a[1] := w + 4.00" );
    assert( a[z].stringify() == "a[z]" );
    assert( ( a[z] == 0.0).stringify() == "a[z] == 0.00" );
    assert( ( a[z] + 0.0).stringify() == "a[z] + 0.00" );
    assert( ( 1 * a[z] ).stringify() == "1.00 * a[z]" );
    assert( ( 0 < a[z] ).stringify() == "0.00 < a[z]" );

    auto c1 = model.addConstraint( x >= 0 );
    assert( c1.stringify() == "x >= 0.00");
    assert( c1._operator == CP::Expression::Operator::greater_or_equal );

    auto c2 = model.addConstraint( x == z );
    assert( c2.stringify() == "x == z");
    assert( c2._operator == CP::Expression::Operator::equal );

    auto c3 = model.addConstraint( true + x <= 3 * z );
    assert( c3.stringify() == "1.00 + x <= 3.00 * z");
    assert( c3._operator == CP::Expression::Operator::less_or_equal );

    auto c4 = model.addConstraint( (y).implies(x >= 4) );
    assert ( c4.stringify() == "( !y ) || ( x >= 4.00 )");
    assert( c4._operator == CP::Expression::Operator::logical_or );

    auto c5 = model.addConstraint( (y == true).implies(x >= 5) );
    assert ( c5.stringify() == "( !( y == 1.00 ) ) || ( x >= 5.00 )");
    assert( CP::isImplication(c5) );
    if ( auto implication = CP::isImplication(c5) ) {
      auto [condition,expression] = implication.value();
      assert( condition.stringify() == "y == 1.00" );
      assert( expression.stringify() == "x >= 5.00" );
    }
    else {
      assert(!"Error");
    }
    std::cout << GREEN << "Variables, expressions, constraints test PASSED" << RESET << std::endl;
  }

  {
    CP::Model model;
    auto& x = model.addRealVariable("x");
    auto& y = model.addIntegerVariable("y");
    auto& z = model.addRealVariable("z");
    auto& w = model.addRealVariable("w");
    auto& v = model.addBinaryVariable("v");
    auto expression = model.addConstraint( x <= min(y, z) );
    CP::Solution solution(model);
    solution.setVariableValue(x,1);
    solution.setVariableValue(y,3);
    solution.setVariableValue(z,2);
    assert( solution.errors().empty() );
    solution.setVariableValue(x,4);
    assert( solution.errors() == "infeasible: x <= min( y, z )");
    solution.setVariableValue(y,4);
    solution.setVariableValue(z,5);
    solution.setVariableValue(v,2);
    assert( !solution.complete() ); // Variable w does not yet have a value
  
//std::cout << "Model: \n" << model.stringify() << std::endl;
//std::cout << "Solution: \n" << solution.stringify() << std::endl;

    solution.setVariableValue(w,0);
    assert( solution.complete() );

    assert( solution.errors().empty() );
    std::cout << GREEN << "Solution test PASSED" << RESET << std::endl;
  }

  // Test custom operator "at" evaluation
  {
    CP::Model model;
    auto& index = model.addIntegerVariable("index");

    // Create deduced variable using at: result := at(index, 10, 20, 30)
    auto atExpr = CP::customOperator("at", index, 10.0, 20.0, 30.0);
    auto& result = model.addVariable(CP::Variable::Type::REAL, "result", atExpr);

    CP::Solution solution(model);
    solution.setVariableValue(index, 1);
    assert( solution.complete() );
    assert( solution.errors().empty() );

    // Verify correctness: at(1, 10, 20, 30) should return 10.0
    auto resultVal = solution.evaluate(result);
    assert( resultVal.has_value() );
    assert( resultVal.value() == 10.0 );
    std::cout << GREEN << "Custom operator 'at' test PASSED" << RESET << std::endl;
  }

  // Test count operator with deduced variables (mimics multiinstanceactivity scenario)
  {
    CP::Model model;
    auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 1.0, 10.0);
    auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 1.0, 10.0);
    auto& z = model.addVariable(CP::Variable::Type::REAL, "z", 1.0, 10.0);

    // Create deduced variable: numElements := count(x, y, z)
    auto countExpr = CP::customOperator("count", x, y, z);
    auto& numElements = model.addVariable(CP::Variable::Type::INTEGER, "numElements", countExpr);

    CP::Solution solution(model);
    solution.setVariableValue(x, 5.0);
    solution.setVariableValue(y, 7.0);
    solution.setVariableValue(z, 3.0);

    // Verify count returns 3 (the number of arguments)
    auto countVal = solution.evaluate(numElements);
    assert( countVal.has_value() );
    assert( countVal.value() == 3.0 );

    // Verify complete() works (this was causing bad_function_call before)
    assert( solution.complete() );
    assert( solution.errors().empty() );
    std::cout << GREEN << "Custom operator 'count' test PASSED" << RESET << std::endl;
  }

  // Test collection lookup in Model with Solution delegation
  {
    // Mock collection registry (simulating BPMNOS::CollectionRegistry)
    std::vector<std::vector<double>> mockCollections = {
      {},                    // index 0: empty
      {10.0, 20.0, 30.0},   // index 1: 3 elements
      {5.0, 15.0}           // index 2: 2 elements
    };

    // Create lookup lambda (caller responsible for bounds)
    auto collectionLookup = [&mockCollections](size_t key) -> const std::vector<double>& {
      return mockCollections[key];
    };

    CP::Model model;

    // Set collection lookup on model only (Solution will delegate to Model)
    // Collections are numbered 0, 1, 2
    model.setCollectionLookup(collectionLookup, mockCollections.size());

    // Test Model::getCollection directly
    const auto& coll0 = model.getCollection(0);
    assert( coll0.size() == 0 );

    const auto& coll1 = model.getCollection(1);
    assert( coll1.size() == 3 );
    assert( coll1[0] == 10.0 );
    assert( coll1[1] == 20.0 );
    assert( coll1[2] == 30.0 );

    const auto& coll2 = model.getCollection(2);
    assert( coll2.size() == 2 );

    // Get collection data for use in constraints (collection at index 1)
    const auto& collection = model.getCollection(1);

    CP::Solution solution(model);

    // Test "at" operator: at(2, 10, 20, 30) returns 20 (1-based indexing)
    auto atExpr = CP::customOperator("at", 2.0, collection[0], collection[1], collection[2]);
    auto& elementValue = model.addVariable(CP::Variable::Type::REAL, "elementValue", atExpr);

    auto elemVal = solution.evaluate(elementValue);
    assert( elemVal.has_value() );
    assert( elemVal.value() == 20.0 );

    // Test "count" operator
    auto countExpr = CP::customOperator("count", collection[0], collection[1], collection[2]);
    auto& numElements = model.addVariable(CP::Variable::Type::INTEGER, "numElements", countExpr);

    auto countVal = solution.evaluate(numElements);
    assert( countVal.has_value() );
    assert( countVal.value() == 3.0 );
    std::cout << GREEN << "Collection lookup test PASSED" << RESET << std::endl;
  }

  // Test Collection struct creates Expression with Operator::collection
  {
    CP::Model model;
    auto& key = model.addIntegerVariable("key");

    // Create collection expression via Collection struct
    CP::Expression collExpr = CP::Collection(key).expression();

    // Verify it has the right operator
    assert(collExpr._operator == CP::Expression::Operator::collection);
    assert(collExpr.operands.size() == 1);

    // Verify the operand is the key variable
    assert(std::holds_alternative<std::reference_wrapper<const CP::Variable>>(collExpr.operands[0]));
    const CP::Variable& varRef = std::get<std::reference_wrapper<const CP::Variable>>(collExpr.operands[0]);
    assert(&varRef == &key);
    std::cout << GREEN << "Collection struct test PASSED" << RESET << std::endl;
  }

  // Test count() free function creates custom operator expression
  {
    CP::Model model;
    auto& key = model.addIntegerVariable("key");

    // Create count(Collection(key)) expression
    CP::Expression countExpr = CP::count(CP::Collection(key));

    // Verify it's a custom operator
    assert(countExpr._operator == CP::Expression::Operator::custom);
    assert(countExpr.operands.size() == 2);  // custom index + collection expression

    // First operand should be the custom operator index for "count"
    assert(std::holds_alternative<size_t>(countExpr.operands[0]));

    // Second operand should be the collection expression
    assert(std::holds_alternative<CP::Expression>(countExpr.operands[1]));
    const CP::Expression& collExpr = std::get<CP::Expression>(countExpr.operands[1]);
    assert(collExpr._operator == CP::Expression::Operator::collection);
    std::cout << GREEN << "Collection count() test PASSED" << RESET << std::endl;
  }

  std::cout << GREEN << "All CP tests passed." << RESET << std::endl;
  return 0;
}

