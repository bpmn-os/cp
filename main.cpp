#include <iostream>
#include <cassert>

#include "cp.h"

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
  assert( y.stringify() == "y ∈ [ 0.00, 1.00 ]");
  auto& z = model.addIntegerVariable("z");
  assert( z.stringify() == "z ∈ [ -infinity, infinity ]");
  
  assert( (x * 3 + z * 5).stringify() == "0.00 + 3.00*x + 5.00*z");
  assert( (3 * x + 5 * z - 4).stringify() == "-4.00 + 3.00*x + 5.00*z");
  assert( (4 + 3 * x + z / 5).stringify() == "4.00 + 3.00*x + 0.20*z");
  assert( ((-x + 5 + 5 * z) / 4).stringify() == "1.25 - 0.25*x + 1.25*z");
  assert( (3 * (x + 2 - 5 * z) / 4).stringify() == "1.50 + 0.75*x - 3.75*z");
  assert( (2 + 3 * (x + 2 + 5 * z / 0.5)).stringify() == "8.00 + 3.00*x + 30.00*z");

  assert( (!y && y).stringify() == "!y && y");
  assert( (y || !y).stringify() == "y || !y");

  assert( CP::max( 0, x, 3 * z ).stringify() == "max{ 0.00, 0.00 + 1.00*x, 0.00 + 3.00*z }");
  assert( CP::min( 0, x, 3 * z ).stringify() == "min{ 0.00, 0.00 + 1.00*x, 0.00 + 3.00*z }");

  assert( CP::if_then_else( y, x, 3 * z ).stringify() == "if y then 0.00 + 1.00*x else 0.00 + 3.00*z");
  assert( CP::n_ary_if( { {y, x}, {!y, 5} }, 3 * z ).stringify() == "if y then 0.00 + 1.00*x else if !y then 5.00 else 0.00 + 3.00*z");

  auto& w = model.addVariable(CP::Variable::Type::BOOLEAN, "w", (y || !y) );
  assert( w.stringify() == "w = y || !y");

  auto c1 = model.addConstraint( x >= 0 );
//std::cout << c1.stringify() << std::endl;
  assert( c1.stringify() == "0.00 + 1.00*x >= 0");
  auto c2 = model.addConstraint( x == z );
  assert( c2.stringify() == "0.00 + 1.00*x - 1.00*z == 0");
  auto c3 = model.addConstraint( 2 * x <= 3 * z );
  assert( c3.stringify() == "0.00 + 2.00*x - 3.00*z <= 0");
  auto c4 = model.addConstraint( x >= 5 + z );
  assert( c4.stringify() == "-5.00 + 1.00*x - 1.00*z >= 0");
  auto c5 = model.addConstraint( 2*x >= 5 + z );
  assert( c5.stringify() == "-5.00 + 2.00*x - 1.00*z >= 0");
  auto c6 = model.addConstraint( x + y >= 5 + z );
  assert( c6.stringify() == "-5.00 + 1.00*x + 1.00*y - 1.00*z >= 0");
  auto c7 = model.addConstraint( z == 5.0 );
  assert( c7.stringify() == "-5.00 + 1.00*z == 0");
  
  return 0;
}

