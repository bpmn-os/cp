#include <iostream>
#include <cassert>

#include "cp.h"
#include "limex_handle.h"

#define GREEN "\033[32m"
#define RESET "\033[0m"

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
    std::cout << GREEN << "LIMEX not_in test PASSED" << RESET << std::endl;
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
    std::cout << GREEN << "LIMEX min test PASSED" << RESET << std::endl;
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
    std::cout << GREEN << "LIMEX collection indexed access test PASSED" << RESET << std::endl;
  }

  {
    CP::Model model;
    auto& z = model.addIntegerVariable("z");
    auto limexExpression = LIMEX::Expression<CP::Expression,CP::Expression>("count(z[])", handle);
    std::vector<CP::Expression> variables = {};
    std::vector< CP::Expression > collectionVariables = { z };
    auto cpExpression = limexExpression.evaluate( variables, collectionVariables );
    assert( cpExpression.stringify() == "count( collection(z) )" );
    std::cout << GREEN << "LIMEX count collection test PASSED" << RESET << std::endl;
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

    std::vector<double> collection0{ 4, 3, 2, 1 };
    std::vector<double> collection1{ 0, 8, 15 };

    model.setCollectionLookup(
      [&collection0,&collection1](double key) -> std::expected<std::vector<double>, std::string> {
        if (key == 0.0) return collection0;
        if (key == 1.0) return collection1;
        return std::unexpected("Collection key not found");
      },
      2  // 2 collections: 0 and 1
    );

    CP::Solution solution(model);

    // x=0: use collection0 = {4, 3, 2, 1}, count=4 (!=3), at(1)=4 (true)
    solution.setVariableValue(x, 0.0);
    assert( !solution.evaluate(constraint1).value() );  // count != 3
    assert( solution.evaluate(constraint2).value() );   // x[1] == 4

    // x=1: use collection1 = {0, 8, 15}, count=3 (==3), at(1)=0 (!=4)
    solution.setVariableValue(x, 1.0);
    assert( solution.evaluate(constraint1).value() );   // count == 3
    assert( !solution.evaluate(constraint2).value() );  // x[1] != 4
    std::cout << GREEN << "LIMEX collection evaluation test PASSED" << RESET << std::endl;
  }

  std::cout << GREEN << "All LIMEX tests passed." << RESET << std::endl;
  return 0;
}
