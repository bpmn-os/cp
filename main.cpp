#include <iostream>
#include <cassert>

#include "cp.h"

#ifdef USE_LIMEX
  #include "limex_handle.h"
#endif 

int main()
{
//  std::cout << ("Create model.") << std::endl;
  {
    CP::Model model;
 
/*
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
*/  
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
    a.emplace_back(0,5);
    a.emplace_back( w + 4 );
    a.emplace_back( a[1] + 5 );
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
//    std::cout << "Model: " << model.stringify() << std::endl;
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
  }
  std::cout << "Basic tests passed." << std::endl;

#ifdef USE_LIMEX
  LIMEX::Handle<CP::Expression,CP::Expression> handle;
  {
    CP::Model model;
    auto& x = model.addRealVariable("x");
    auto& y = model.addIntegerVariable("y");
    auto& z = model.addRealVariable("z");
    auto limexExpression = LIMEX::Expression<CP::Expression,CP::Expression>("z not in {3, abs(x), y + 5}", handle);
    auto cpExpression = limexExpression.evaluate({z, x, y});
  assert( cpExpression.stringify() == "n_ary_if( z == 3.00, 0.00, z == if_then_else( x >= 0.00, x, -x ), 0.00, z == y + 5.00, 0.00, 1.00 )" );
  }
  
  {
    CP::Model model;
    auto& x = model.addRealVariable("x");
    auto& y = model.addIntegerVariable("y");
    auto limexExpression = LIMEX::Expression<CP::Expression,CP::Expression>("min{3, x, y + 5}", handle);
    std::vector<CP::Expression> variables = {x, y};
    std::vector< CP::Expression > collectionVariables = {};
    auto cpExpression = limexExpression.evaluate( variables, collectionVariables );
    assert( cpExpression.stringify() == "min( 3.00, x, y + 5.00 )" );
  }
  
  {
    CP::Model model;
    auto& w = model.addRealVariable("w");
    auto& v = model.addIntegerVariable("v");
    auto& z = model.addIntegerVariable("z");
    auto limexExpression = LIMEX::Expression<CP::Expression,CP::Expression>("w := z[v]", handle);
    assert( !limexExpression.getVariables().empty() && limexExpression.getVariables().front() == v.name );
    assert( !limexExpression.getCollections().empty() && limexExpression.getCollections().front() == z.name );
    assert( limexExpression.getTarget() && limexExpression.getTarget().value() == w.name );
  
    std::vector<CP::Expression> variables = {v};
    std::vector< CP::Expression > collectionVariables = { z };
    auto cpExpression = limexExpression.evaluate( variables, collectionVariables );
    assert( cpExpression.stringify() == "collection(z)[v]" );
  }
  
  {
    CP::Model model;
    auto& z = model.addIntegerVariable("z");
    auto limexExpression = LIMEX::Expression<CP::Expression,CP::Expression>("count(z[])", handle);
    std::vector<CP::Expression> variables = {};
    std::vector< CP::Expression > collectionVariables = { z };
    auto cpExpression = limexExpression.evaluate( variables, collectionVariables );
    assert( cpExpression.stringify() == "count( collection(z) )" );
  }
  
  {
    CP::Model model;
    auto& x = model.addIntegerVariable("x");
    std::vector<CP::Expression> variables = {};
    std::vector< CP::Expression > collections = { x };

    auto limexExpression1 = LIMEX::Expression<CP::Expression,CP::Expression>("count(x[]) == 3", handle);
    auto cpExpression1 = limexExpression1.evaluate( variables, collections );
    auto& constraint1 = model.addConstraint(cpExpression1);

    auto limexExpression2 = LIMEX::Expression<CP::Expression,CP::Expression>("x[1] == 4", handle);
    auto cpExpression2 = limexExpression2.evaluate( variables, collections );
    auto& constraint2 = model.addConstraint(cpExpression2);

//std::cout << "Model: " << model.stringify() << std::endl;
    CP::Solution solution(model);
  
    std::vector<double> collection1{ 4, 3, 2, 1 };
    std::vector<double> collection2{ 0, 8, 15 };
       
    solution.setCollectionEvaluator( 
      [&collection1,&collection2](double value) -> std::expected< std::reference_wrapper<const std::vector<double> >, std::string > {
        return ( value == 42 ? collection1 : collection2 );
      }
    );


    solution.setVariableValue(x,42);
//std::cout << "Solution: " << solution.stringify() << std::endl;
    assert( !solution.evaluate(constraint1).value() );
    assert( solution.evaluate(constraint2).value() );

    solution.setVariableValue(x,15);
//std::cout << "Solution: " << solution.stringify() << std::endl;
    assert( solution.evaluate(constraint1).value() );
    assert( !solution.evaluate(constraint2).value() );
  }
  std::cout << "LIMEX tests passed." << std::endl;
#endif 
  return 0;
}

