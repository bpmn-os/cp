#include <iostream>
#include <cassert>
#include <optimizer/hexalyoptimizer.h>

#define GREEN "\033[32m"
#define RESET "\033[0m"

int main() {
    // Simple test: minimize x + y subject to x + y >= 10, x,y in [0,100]
    hexaly::HexalyOptimizer optimizer;
    hexaly::HxModel model = optimizer.getModel();

    // Create integer variables
    hexaly::HxExpression x = model.intVar(0, 100);
    hexaly::HxExpression y = model.intVar(0, 100);

    // Constraint: x + y >= 10
    model.constraint(x + y >= 10);

    // Objective: minimize x + y
    model.minimize(x + y);

    // Close model
    model.close();

    // Solve with 5 second time limit
    optimizer.getParam().setTimeLimit(5);
    optimizer.solve();

    // Get solution
    hexaly::HxSolution solution = optimizer.getSolution();
    hexaly::HxSolutionStatus status = solution.getStatus();

    // Verify we found a solution
    assert(status == hexaly::SS_Optimal || status == hexaly::SS_Feasible);

    // Get values
    long xVal = solution.getIntValue(x);
    long yVal = solution.getIntValue(y);

    // Verify constraint is satisfied
    assert(xVal + yVal >= 10);

    // For minimization, optimal should be x + y = 10
    assert(xVal + yVal == 10);

    std::cout << GREEN << "Hexaly dummy model test PASSED" << RESET << std::endl;
    std::cout << "  x = " << xVal << ", y = " << yVal << std::endl;

    return 0;
}
