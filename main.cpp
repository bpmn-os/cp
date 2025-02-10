#include <iostream>
#include <cassert>

#include "cp.h"

#define USE_LIMEX
#ifdef USE_LIMEX
  #include "limex_callables.h"
#endif 

int main()
{
  std::cout << ("Create model.") << std::endl;
  CP::Model model;
  
  auto objectiveSense = model.getObjectiveSense();
  if ( objectiveSense == CP::Model::ObjectiveSense::FEASIBLE ) {
    std::cout << ("Objective is to find a feasible solution.") << std::endl;
  }
  else if ( objectiveSense == CP::Model::ObjectiveSense::MINIMIZE ) {
    std::cout << ("Objective is to find a solution that minimizes the objective.") << std::endl;
  }
  else if ( objectiveSense == CP::Model::ObjectiveSense::MAXIMIZE ) {
    std::cout << ("Objective is to find a solution that maximizes the objective.") << std::endl;
  }
  
  auto& x = model.addRealVariable("x");
  assert( x.stringify() == "x ∈ [ -infinity, infinity ]");
  auto& y = model.addBinaryVariable("y");
  assert( y.stringify() == "y ∈ { false, true }");
  auto& z = model.addIntegerVariable("z");
  assert( z.stringify() == "z ∈ { -infinity, ..., infinity }");

  assert( (x * 3 + z * 5).stringify() == "( x * 3.00 ) + ( z * 5.00 )");
  assert( (3 * x + 5 * z - 4).stringify() == "( ( 3.00 * x ) + ( 5.00 * z ) ) - 4.00");
  assert( (4 + 3 * x + z / 5).stringify() == "( 4.00 + ( 3.00 * x ) ) + ( z / 5.00 )");
  assert( (4 + 2 + 3*3 * x + z / 5 * 5).stringify() == "( 6.00 + ( 9.00 * x ) ) + ( ( z / 5.00 ) * 5.00 )");

  assert( (!y && y).stringify() == "( !y ) && y");
  assert( (y || !y).stringify() == "y || ( !y )");
  
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
  assert( s.size() == 3);
  assert( s[0].name == "s[0]");
  assert( s[1].name == "s[1]");
  assert( s[2].name == "s[2]");
  assert( model.getSequences().back().stringify() == "( s[0], s[1], s[2] ) is permutation of { 1, ..., 3 }");

  auto& a = model.addIndexedVariables(CP::Variable::Type::INTEGER, "a");
  a.emplace_back(0,5);
  a.emplace_back( w + 4 );
  a.emplace_back( a[1] + 5 );
  assert( model.getIndexedVariables().back().stringify() == "a := { a[0] ∈ { 0, ..., 5 }, a[1] := w + 4.00, a[2] := a[1] + 5.00 }" );
  assert( a[1].stringify() == "a[1] := w + 4.00" );
  assert( a[z].stringify() == "a[z]" );

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
  std::cout << model.stringify() << std::endl;


#ifdef USE_LIMEX

  LIMEX::Callables<CP::Expression> callables;
  auto l1 = LIMEX::Expression<CP::Expression>("z not in {3, abs(x), y + 5}", callables);
//std::cout << "LIMEX: " << l1.stringify() << std::endl;
  auto e1 = l1.evaluate({z, x, y});
//std::cout << "CP: " << e1.stringify() << std::endl;
  assert( e1.stringify() == "n_ary_if( z == 3.00, 0.00, z == if_then_else( x >= 0.00, x, -( x ) ), 0.00, z == ( y ) + ( 5.00 ), 0.00, 1.00 )" );

  auto l2 = LIMEX::Expression<CP::Expression>("min{3, x, y + 5}", callables);
  auto e2 = l2.evaluate( {x, y} );
//std::cout << "CP: " << e2.stringify() << std::endl;
  assert( e2.stringify() == "min( 3.00, x, ( y ) + ( 5.00 ) )" );

  auto l3 = LIMEX::Expression<CP::Expression>("w := z[v]", callables);
//std::cout << "LIMEX: " << l3.stringify() << std::endl;
//for ( auto variable : l3.getVariables() ) std::cerr << variable << std::endl; 
  assert( !l3.getVariables().empty() && l3.getVariables().front() == "v" );
  assert( l3.getTarget() && l3.getTarget().value() == "w" );
  auto e3 = l3.evaluate({v},{ {x, y} });
//std::cout << "CP: " << e3.stringify() << std::endl;
  assert( e3.stringify() == "n_ary_if( v == 1.00, x, v == 2.00, y, 0.00 )" );
#endif 


  CP::Model solvedModel;
  auto& X = solvedModel.addRealVariable("X");
  auto& Y = solvedModel.addRealVariable("Y");
  auto& Z = solvedModel.addRealVariable("Z");
  auto expression = solvedModel.addConstraint( X <= min(Y, Z) );
  CP::Solution solution(solvedModel);
  solution.setVariableValue(X,1);
  solution.setVariableValue(Y,3);
  solution.setVariableValue(Z,2);
  assert( solution.validate() == "missing objective, expected: " + std::format("{:.6f}", 0.0));
  solution.setVariableValue(X,4);
  solution.setObjectiveValue(1);
  assert( solution.validate() == "infeasible: X <= min( Y, Z )\nwrong objective, expected: " + std::format("{:.6f}", 0.0));
  solution.setVariableValue(Y,4);
  solution.setVariableValue(Z,5);
  solution.setObjectiveValue(0);
  assert( solution.validate() == "objective: " + std::format("{:.6f}", 0.0));

  return 0;
}

