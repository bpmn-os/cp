#include <iostream>
#include <cassert>

#include "cp.h"
#include "limex_handle.h"

int main()
{
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

    std::vector<double> collection1{ 4, 3, 2, 1 };
    std::vector<double> collection2{ 0, 8, 15 };

    model.setCollectionLookup(
      [&collection1,&collection2](double value) -> std::expected<std::vector<double>, std::string> {
        return ( value == 42 ? collection1 : collection2 );
      }
    );

    CP::Solution solution(model);

    solution.setVariableValue(x,42);
    assert( !solution.evaluate(constraint1).value() );
    assert( solution.evaluate(constraint2).value() );

    solution.setVariableValue(x,15);
    assert( solution.evaluate(constraint1).value() );
    assert( !solution.evaluate(constraint2).value() );
  }

  std::cout << "LIMEX tests passed." << std::endl;
  return 0;
}
