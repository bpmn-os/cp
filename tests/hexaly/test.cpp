#include "cp.h"
#include "hexaly/hexaly_adapter.h"
#include <iostream>
#include <cassert>
#include <cmath>

#define GREEN "\033[32m"
#define RESET "\033[0m"

int main() {
    int testNum = 0;
    // Test: Single integer variable
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        model.addConstraint(x == 42.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(solver.getName() == "Hexaly");
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 42.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Single integer variable" << RESET << std::endl;
    }
    // Test: Multiple variable types
    {
        CP::Model model;
        const auto& b = model.addBinaryVariable("b");
        const auto& i = model.addIntegerVariable("i");
        const auto& r = model.addRealVariable("r");

        model.addConstraint(b == 1.0);
        model.addConstraint(i == 5.0);
        model.addConstraint(r == 3.14);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto bVal = solver.getSolution()->getVariableValue(b);
        auto iVal = solver.getSolution()->getVariableValue(i);
        auto rVal = solver.getSolution()->getVariableValue(r);

        assert(bVal.has_value() && iVal.has_value() && rVal.has_value());
        assert(std::abs(bVal.value() - 1.0) < 1e-5);
        assert(std::abs(iVal.value() - 5.0) < 1e-5);
        assert(std::abs(rVal.value() - 3.14) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Multiple variable types" << RESET << std::endl;
    }
    // Test: Indexed variables
    {
        CP::Model model;
        auto& vars = model.addIndexedVariables(CP::Variable::Type::INTEGER, "x");
        model.addIndexedVariable(vars, 0, 10);
        model.addIndexedVariable(vars, 5, 15);
        model.addIndexedVariable(vars, -5, 5);

        model.addConstraint(vars[0] == 5.0);
        model.addConstraint(vars[1] == 10.0);
        model.addConstraint(vars[2] == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();

        assert(std::abs(solver.getSolution()->getVariableValue(vars[0]).value() - 5.0) < 1e-5);
        assert(std::abs(solver.getSolution()->getVariableValue(vars[1]).value() - 10.0) < 1e-5);
        assert(std::abs(solver.getSolution()->getVariableValue(vars[2]).value() - 0.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Indexed variables" << RESET << std::endl;
    }
    // Test: Sequence with alldifferent
    {
        CP::Model model;
        const auto& seq = model.addSequence("seq", 5);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto seqVals = solver.getSolution()->getSequenceValues(seq);
        assert(seqVals.has_value());
        assert(seqVals.value().size() == 5);

        // Check all values are different and in range [1, 5]
        std::vector<bool> seen(6, false);
        for (auto val : seqVals.value()) {
            assert(val >= 1 && val <= 5);
            assert(!seen[val]);
            seen[val] = true;
        }

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Sequence with alldifferent" << RESET << std::endl;
    }
    // Test: Minimize objective
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        model.setObjective(x);
        model.addConstraint(x >= 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Minimize objective" << RESET << std::endl;
    }
    // Test: Maximize objective
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        model.setObjective(x);
        model.addConstraint(x <= 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Maximize objective" << RESET << std::endl;
    }
    // Test: Linear objective with coefficients (2*x + 3*y)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        model.setObjective(2.0 * x + 3.0 * y);
        model.addConstraint(x + y >= 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        // Optimal: x=10, y=0 (objective = 20)
        assert(std::abs(xVal.value() - 10.0) < 1e-5);
        assert(std::abs(yVal.value() - 0.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Linear objective with coefficients" << RESET << std::endl;
    }
    // Test: Linear objective with subtraction (x - 2*y)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);
        model.setObjective(x - 2.0 * y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        // Optimal: x=10, y=0 (objective = 10)
        assert(std::abs(xVal.value() - 10.0) < 1e-5);
        assert(std::abs(yVal.value() - 0.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Linear objective with subtraction" << RESET << std::endl;
    }
    // Test: Negated variable (-x)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        model.setObjective(-x);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        // Minimize -x => maximize x => x=10
        assert(std::abs(xVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Negated variable objective" << RESET << std::endl;
    }
    // Test: Negate in constraint (-x == -5 means x == 5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        model.addConstraint(-x == -5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Negate in constraint" << RESET << std::endl;
    }
    // Test: Feasibility (no objective)
    {
        CP::Model model(CP::Model::ObjectiveSense::FEASIBLE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);

        model.addConstraint(x + y == 7.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() + yVal.value() - 7.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Feasibility problem" << RESET << std::endl;
    }
    // Test: Simple equality constraint (x == 5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        model.addConstraint(x == 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Equality constraint (feasible)" << RESET << std::endl;
    }
    // Test: Equality constraint (infeasible: x == 5 with x=6)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 6, 6);
        model.addConstraint(x == 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Equality constraint (infeasible)" << RESET << std::endl;
    }
    // Test: Simple inequality (x <= 10)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        model.setObjective(x);
        model.addConstraint(x <= 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() <= 10.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Less-or-equal constraint (feasible)" << RESET << std::endl;
    }
    // Test: Less-or-equal constraint (infeasible: x <= 5 with x=6)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 6, 6);
        model.addConstraint(x <= 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Less-or-equal constraint (infeasible)" << RESET << std::endl;
    }
    // Test: Greater-or-equal (x >= 0)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", -10, 100);
        model.setObjective(x);
        model.addConstraint(x >= 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() >= 0.0 - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Greater-or-equal constraint (feasible)" << RESET << std::endl;
    }
    // Test: Greater-or-equal constraint (infeasible: x >= 5 with x=4)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 4, 4);
        model.addConstraint(x >= 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Greater-or-equal constraint (infeasible)" << RESET << std::endl;
    }
    // Test: Linear constraint (2*x + 3*y <= 15)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);
        model.setObjective(x + y);
        model.addConstraint(2.0 * x + 3.0 * y <= 15.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(2.0 * xVal.value() + 3.0 * yVal.value() <= 15.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Linear constraint" << RESET << std::endl;
    }
    // Test: Equality with two variables (x + y == 10)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);
        model.addConstraint(x + y == 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() + yVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Two-variable equality" << RESET << std::endl;
    }
    // Test: Multiple constraints
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        model.setObjective(x + y);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);
        model.addConstraint(x + y <= 100.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() + yVal.value() <= 100.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Multiple constraints" << RESET << std::endl;
    }
    // Test: Constraint with subtraction (x - y >= 5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 20);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 20);
        model.addConstraint(x - y >= 5.0);
        model.addConstraint(x == 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() - yVal.value() >= 5.0 - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Subtraction constraint" << RESET << std::endl;
    }
    // Test: Constraint with constant on left (10 <= x + y)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        model.setObjective(x + y);
        model.addConstraint(10.0 <= x + y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() + yVal.value() >= 10.0 - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Constant on left side" << RESET << std::endl;
    }
    // Test: Solve simple problem (minimize x, x >= 5)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        model.setObjective(x);
        model.addConstraint(x >= 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Minimize with lower bound" << RESET << std::endl;
    }
    // Test: Maximize problem (maximize x, x <= 10)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        model.setObjective(x);
        model.addConstraint(x <= 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Maximize with upper bound" << RESET << std::endl;
    }
    // Test: Two variables (minimize x + y, x >= 2, y >= 3)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        model.setObjective(x + y);
        model.addConstraint(x >= 2.0);
        model.addConstraint(y >= 3.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() - 2.0) < 1e-5);
        assert(std::abs(yVal.value() - 3.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Two variables minimize" << RESET << std::endl;
    }
    // Test: Linear programming (minimize 2*x + 3*y, x + y >= 10, x >= 0, y >= 0)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        model.setObjective(2.0 * x + 3.0 * y);
        model.addConstraint(x + y >= 10.0);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        // Optimal: x=10, y=0 (objective = 20)
        assert(std::abs(xVal.value() - 10.0) < 1e-5);
        assert(std::abs(yVal.value() - 0.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Linear programming" << RESET << std::endl;
    }
    // Test: Equality constraint (x + y == 7, minimize x, x >= 0, y >= 0)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        model.setObjective(x);
        model.addConstraint(x + y == 7.0);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        // Optimal: x=0, y=7
        assert(std::abs(xVal.value() - 0.0) < 1e-5);
        assert(std::abs(yVal.value() - 7.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Equality constraint" << RESET << std::endl;
    }
    // Test: Non-linear constraint (x * y <= 10)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 1, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 1, 10);
        model.setObjective(x + y);
        model.addConstraint(x * y <= 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() * yVal.value() <= 10.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Non-linear constraint" << RESET << std::endl;
    }
    // Test: Solve non-linear (minimize x+y, x*y >= 12, x >= 1, y >= 1)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 1, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 1, 100);
        model.setObjective(x + y);
        model.addConstraint(x * y >= 12.0);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        double product = xVal.value() * yVal.value();
        assert(product >= 12.0 - 1e-5);
        assert(xVal.value() + yVal.value() <= 8.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Solve non-linear problem" << RESET << std::endl;
    }
    // Test: Logical NOT constraint (!x == 0 => x must be 1)
    {
        CP::Model model;
        const auto& x = model.addBinaryVariable("x");
        model.addConstraint(!x == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 1.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical NOT" << RESET << std::endl;
    }
    // Test: Logical AND constraint (x && y == 1)
    {
        CP::Model model;
        const auto& x = model.addBinaryVariable("x");
        const auto& y = model.addBinaryVariable("y");
        model.addConstraint((x && y) == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() - 1.0) < 1e-5);
        assert(std::abs(yVal.value() - 1.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical AND" << RESET << std::endl;
    }
    // Test: Logical OR constraint (x || y == 1, x == 0)
    {
        CP::Model model;
        const auto& x = model.addBinaryVariable("x");
        const auto& y = model.addBinaryVariable("y");
        model.addConstraint((x || y) == 1.0);
        model.addConstraint(x == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() - 0.0) < 1e-5);
        assert(std::abs(yVal.value() - 1.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR" << RESET << std::endl;
    }
    // Test: Custom operator sum
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        const auto& z = model.addVariable(CP::Variable::Type::INTEGER, "z", 0, 100);

        auto sumExpr = CP::customOperator("sum", x, y, z);
        model.setObjective(sumExpr);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 2.0);
        model.addConstraint(z >= 3.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        auto zVal = solver.getSolution()->getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());
        assert(std::abs(xVal.value() - 1.0) < 1e-5);
        assert(std::abs(yVal.value() - 2.0) < 1e-5);
        assert(std::abs(zVal.value() - 3.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator sum" << RESET << std::endl;
    }
    // Test: Custom operator pow (x^2 == 16)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);

        auto powExpr = CP::customOperator("pow", x, 2.0);
        model.addConstraint(powExpr == 16.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 4.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator pow" << RESET << std::endl;
    }
    // Test: Custom operator log (log(1) == 0)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.1, 10.0);

        auto logExpr = CP::customOperator("log", x);
        model.addConstraint(logExpr == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 1.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator log" << RESET << std::endl;
    }
    // Test: Custom operator exp (exp(0) == 1)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", -5.0, 5.0);

        auto expExpr = CP::customOperator("exp", x);
        model.addConstraint(expExpr == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 0.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator exp" << RESET << std::endl;
    }
    // Test: Custom operator min
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 5, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 3, 100);

        auto minExpr = CP::customOperator("min", x, y);
        model.setObjective(minExpr);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        double minVal = std::min(xVal.value(), yVal.value());
        assert(std::abs(minVal - 3.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator min" << RESET << std::endl;
    }
    // Test: Custom operator max
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 10, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 7, 100);
        const auto& z = model.addVariable(CP::Variable::Type::INTEGER, "z", 5, 100);

        auto maxExpr = CP::customOperator("max", x, y, z);
        model.setObjective(maxExpr);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        auto zVal = solver.getSolution()->getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());
        double maxVal = std::max({xVal.value(), yVal.value(), zVal.value()});
        assert(std::abs(maxVal - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator max" << RESET << std::endl;
    }
    // Test: n_ary_if operator
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& selector = model.addBinaryVariable("selector");

        CP::Cases cases = {
            {selector, 10.0}
        };
        auto ifExpr = CP::n_ary_if(cases, 5.0);
        model.addConstraint(x == ifExpr);
        model.setObjective(x);
        model.addConstraint(selector == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto selectorVal = solver.getSolution()->getVariableValue(selector);
        assert(xVal.has_value() && selectorVal.has_value());
        assert(std::abs(selectorVal.value() - 1.0) < 1e-5);
        assert(std::abs(xVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: n_ary_if operator" << RESET << std::endl;
    }
    // Test: n_ary_if with multiple mutually exclusive conditions
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 500);
        const auto& c1 = model.addBinaryVariable("c1");
        const auto& c2 = model.addBinaryVariable("c2");
        const auto& c3 = model.addBinaryVariable("c3");

        // if c1 then 100, else if c2 then 200, else if c3 then 300, else 400
        CP::Cases cases = {
            {c1, 100.0},
            {c2, 200.0},
            {c3, 300.0}
        };
        auto ifExpr = CP::n_ary_if(cases, 400.0);
        model.addConstraint(x == ifExpr);
        model.setObjective(x);

        // Test case: c2 is true, others false (mutually exclusive)
        model.addConstraint(c1 == 0.0);
        model.addConstraint(c2 == 1.0);
        model.addConstraint(c3 == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());

        // c2=1, so x should be 200
        assert(std::abs(xVal.value() - 200.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: n_ary_if with multiple conditions" << RESET << std::endl;
    }
    // Test: n_ary_if else case (no condition true)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 500);
        const auto& c1 = model.addBinaryVariable("c1");
        const auto& c2 = model.addBinaryVariable("c2");

        // if c1 then 100, else if c2 then 200, else 300
        CP::Cases cases = {
            {c1, 100.0},
            {c2, 200.0}
        };
        auto ifExpr = CP::n_ary_if(cases, 300.0);
        model.addConstraint(x == ifExpr);
        model.setObjective(x);

        // Test case: no condition true (else case)
        model.addConstraint(c1 == 0.0);
        model.addConstraint(c2 == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());

        // No condition true, so x should be 300 (else value)
        assert(std::abs(xVal.value() - 300.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: n_ary_if else case" << RESET << std::endl;
    }
    // Test: if_then_else operator
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& condition = model.addBinaryVariable("condition");

        auto ifExpr = CP::if_then_else(condition, 20.0, 8.0);
        model.addConstraint(x == ifExpr);
        model.setObjective(x);
        model.addConstraint(condition == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto conditionVal = solver.getSolution()->getVariableValue(condition);
        assert(xVal.has_value() && conditionVal.has_value());
        assert(std::abs(conditionVal.value() - 0.0) < 1e-5);
        assert(std::abs(xVal.value() - 8.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: if_then_else operator" << RESET << std::endl;
    }
    // Test: Sequence with specific ordering
    {
        CP::Model model;
        const auto& seq = model.addSequence("perm", 4);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto seqVals = solver.getSolution()->getSequenceValues(seq);
        assert(seqVals.has_value());

        std::vector<bool> seen(5, false);
        for (auto val : seqVals.value()) {
            assert(val >= 1 && val <= 4);
            int intVal = static_cast<int>(val);
            assert(!seen[intVal]);
            seen[intVal] = true;
        }

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Sequence with alldifferent" << RESET << std::endl;
    }
    // Test: Indexed variables (element constraint)
    {
        CP::Model model;
        auto& arr = model.addIndexedVariables(CP::Variable::Type::INTEGER, "arr");
        model.addIndexedVariable(arr, 0, 10);
        model.addIndexedVariable(arr, 0, 10);
        model.addIndexedVariable(arr, 0, 10);

        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 0, 2);
        const auto& result_var = model.addIntegerVariable("result");

        model.addConstraint(result_var == arr[index]);
        model.addConstraint(arr[0] == 5.0);
        model.addConstraint(arr[1] == 7.0);
        model.addConstraint(arr[2] == 3.0);
        model.addConstraint(index == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto indexVal = solver.getSolution()->getVariableValue(index);
        auto resultVal = solver.getSolution()->getVariableValue(result_var);
        assert(indexVal.has_value() && resultVal.has_value());
        assert(std::abs(indexVal.value() - 1.0) < 1e-5);
        assert(std::abs(resultVal.value() - 7.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Indexed variables (element constraint)" << RESET << std::endl;
    }
    // Test: Custom operator at
    {
        CP::Model model;
        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 1, 3);
        const auto& result_var = model.addIntegerVariable("result");

        auto atExpr = CP::customOperator("at", index, 10.0, 20.0, 30.0);
        model.addConstraint(result_var == atExpr);
        model.addConstraint(index == 2.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto indexVal = solver.getSolution()->getVariableValue(index);
        auto resultVal = solver.getSolution()->getVariableValue(result_var);
        assert(indexVal.has_value() && resultVal.has_value());
        assert(std::abs(indexVal.value() - 2.0) < 1e-5);
        assert(std::abs(resultVal.value() - 20.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator at" << RESET << std::endl;
    }
    // Test: Not-equal constraint (feasible)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 5, 5);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 3, 7);

        model.addConstraint(x != y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);
        assert(std::abs(yVal.value() - 5.0) > 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Not-equal constraint (feasible)" << RESET << std::endl;
    }
    // Test: Not-equal constraint (infeasible: x != 5 with x=5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 5, 5);
        model.addConstraint(x != 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Not-equal constraint (infeasible)" << RESET << std::endl;
    }
    // Test: Less-than constraint (infeasible: x < y with x=5, y=5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 5, 5);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 5, 5);
        model.addConstraint(x < y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Less-than constraint (infeasible)" << RESET << std::endl;
    }
    // Test: Less-than constraint (feasible: x < y with x=5, y=6)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 5, 5);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 6, 6);
        model.addConstraint(x < y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);
        assert(std::abs(yVal.value() - 6.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Less-than constraint (feasible)" << RESET << std::endl;
    }
    // Test: Greater-than constraint (infeasible: x > y with x=5, y=5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 5, 5);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 5, 5);
        model.addConstraint(x > y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Greater-than constraint (infeasible)" << RESET << std::endl;
    }
    // Test: Greater-than constraint (feasible: x > y with x=6, y=5)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 6, 6);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 5, 5);
        model.addConstraint(x > y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() - 6.0) < 1e-5);
        assert(std::abs(yVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Greater-than constraint (feasible)" << RESET << std::endl;
    }
    // Test: Division operator
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 12, 20);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 1, 3);

        model.setObjective(x / y);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        // Optimal: x=12, y=3, x/y = 4
        assert(std::abs(xVal.value() - 12.0) < 1e-5);
        assert(std::abs(yVal.value() - 3.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Division operator" << RESET << std::endl;
    }
    // Test: Simple deduced variable (b := a)
    {
        CP::Model model;
        const auto& a = model.addVariable(CP::Variable::Type::BOOLEAN, "a", 0.0, 1.0);
        const auto& b = model.addVariable(CP::Variable::Type::BOOLEAN, "b", a);

        model.addConstraint(a == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto aVal = solver.getSolution()->getVariableValue(a);
        auto bVal = solution->evaluate(b);

        assert(aVal.has_value());
        assert(bVal.has_value());
        assert(std::abs(aVal.value() - 1.0) < 1e-5);
        assert(std::abs(bVal.value() - 1.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Simple deduced variable" << RESET << std::endl;
    }
    // Test: Deduced variable with arithmetic expression (c := a + b)
    {
        CP::Model model;
        const auto& a = model.addVariable(CP::Variable::Type::REAL, "a", 0.0, 10.0);
        const auto& b = model.addVariable(CP::Variable::Type::REAL, "b", 0.0, 10.0);
        const auto& c = model.addVariable(CP::Variable::Type::REAL, "c", a + b);

        model.addConstraint(a == 3.0);
        model.addConstraint(b == 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto aVal = solver.getSolution()->getVariableValue(a);
        auto bVal = solver.getSolution()->getVariableValue(b);
        auto cVal = solution->evaluate(c);

        assert(aVal.has_value() && bVal.has_value() && cVal.has_value());
        assert(std::abs(aVal.value() - 3.0) < 1e-5);
        assert(std::abs(bVal.value() - 5.0) < 1e-5);
        assert(std::abs(cVal.value() - 8.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Deduced variable with arithmetic expression" << RESET << std::endl;
    }
    // Test: Deduced variable from IndexedVariable access
    {
        CP::Model model;

        auto& array = model.addIndexedVariables(CP::Variable::Type::REAL, "array");
        model.addIndexedVariable(array, 5.0, 5.0);
        model.addIndexedVariable(array, 10.0, 10.0);
        model.addIndexedVariable(array, 15.0, 15.0);

        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 0.0, 2.0);
        const auto& value = model.addVariable(CP::Variable::Type::REAL, "value", array[index]);

        model.addConstraint(index == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto indexVal = solver.getSolution()->getVariableValue(index);
        auto valueVal = solution->evaluate(value);

        assert(indexVal.has_value());
        assert(valueVal.has_value());
        assert(std::abs(indexVal.value() - 1.0) < 1e-5);
        assert(std::abs(valueVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Deduced variable from IndexedVariable access" << RESET << std::endl;
    }
    // Test: Unbounded variables
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        model.setObjective(x + y);
        model.addConstraint(x + y >= 10.0);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(std::abs(xVal.value() + yVal.value() - 10.0) < 1e-5);
        assert(xVal.value() >= 0.0 - 1e-5);
        assert(yVal.value() >= 0.0 - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Unbounded variables" << RESET << std::endl;
    }
    // Test: Logical OR constraint (!a || (x >= y))
    {
        CP::Model model;
        const auto& a = model.addBinaryVariable("a");
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);

        model.addConstraint(!a || (x >= y));
        model.addConstraint(a == 1.0);
        model.addConstraint(y == 5.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() >= yVal.value() - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR constraint (!a || (x >= y))" << RESET << std::endl;
    }
    // Test: Logical OR with <= comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint(y == 10.0);
        model.addConstraint(!flag || (x <= y));
        model.addConstraint(flag == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() <= yVal.value() + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR with <= comparison" << RESET << std::endl;
    }
    // Test: Logical OR with == comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);

        model.addConstraint(!flag || (x == 42.0));
        model.addConstraint(flag == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 42.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR with == comparison" << RESET << std::endl;
    }
    // Test: AND of two comparisons
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint((x >= 10.0) && (y <= 20.0));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() >= 10.0 - 1e-5);
        assert(yVal.value() <= 20.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: AND of two comparisons" << RESET << std::endl;
    }
    // Test: OR of two comparisons
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 5.0, 95.0);

        model.addConstraint((x <= 10.0) || (x >= 90.0));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() <= 10.0 + 1e-5 || xVal.value() >= 90.0 - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: OR of two comparisons" << RESET << std::endl;
    }
    // Test: Negation of comparison
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);

        model.addConstraint(!(x >= 50.0));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() < 50.0 + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Negation of comparison" << RESET << std::endl;
    }
    // Test: Custom operator avg
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 100);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 100);
        const auto& z = model.addVariable(CP::Variable::Type::INTEGER, "z", 0, 100);

        // Minimize avg(x, y, z) = (x + y + z) / 3
        auto avgExpr = CP::customOperator("avg", x, y, z);
        model.setObjective(avgExpr);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 2.0);
        model.addConstraint(z >= 3.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        auto zVal = solver.getSolution()->getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());

        // Optimal: x=1, y=2, z=3, avg = 6/3 = 2
        assert(std::abs(xVal.value() - 1.0) < 1e-5);
        assert(std::abs(yVal.value() - 2.0) < 1e-5);
        assert(std::abs(zVal.value() - 3.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator avg" << RESET << std::endl;
    }
    // Test: Custom operator count
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 1.0, 10.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 1.0, 10.0);
        const auto& z = model.addVariable(CP::Variable::Type::REAL, "z", 1.0, 10.0);

        // count(x, y, z) should return 3 (the number of arguments)
        auto countExpr = CP::customOperator("count", x, y, z);
        const auto& result_var = model.addVariable(CP::Variable::Type::REAL, "result", countExpr);

        CP::HexalySolver solver(model);
        auto solve_result = solver.solve(5.0);

        assert(solve_result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto resultVal = solution->evaluate(result_var);

        assert(resultVal.has_value());
        assert(std::abs(resultVal.value() - 3.0) < 1e-5); // count of 3 args = 3

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Custom operator count" << RESET << std::endl;
    }
    // Test: Logical NOT with non-boolean value (!5 should be 0)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);

        // !x == 0 when x != 0
        model.addConstraint(x == 5.0);
        model.addConstraint((!x) == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical NOT with non-boolean value" << RESET << std::endl;
    }
    // Test: Logical AND with non-boolean values (3 && 5 should be 1)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);
        const auto& z = model.addVariable(CP::Variable::Type::INTEGER, "z", 0, 10);

        // Set x=3, y=5, then (x && y) should equal 1
        model.addConstraint(x == 3.0);
        model.addConstraint(y == 5.0);
        model.addConstraint((x && y) == 1.0);
        model.addConstraint(z == 7.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        auto zVal = solver.getSolution()->getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());
        assert(std::abs(xVal.value() - 3.0) < 1e-5);
        assert(std::abs(yVal.value() - 5.0) < 1e-5);
        assert(std::abs(zVal.value() - 7.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical AND with non-boolean values" << RESET << std::endl;
    }
    // Test: Logical OR with non-boolean values (0 || 7 should be 1)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);
        const auto& z = model.addVariable(CP::Variable::Type::INTEGER, "z", 0, 10);

        // Set x=0, y=7, then (x || y) should equal 1
        model.addConstraint(x == 0.0);
        model.addConstraint(y == 7.0);
        model.addConstraint((x || y) == 1.0);
        model.addConstraint(z == 2.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        auto zVal = solver.getSolution()->getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());
        assert(std::abs(xVal.value() - 0.0) < 1e-5);
        assert(std::abs(yVal.value() - 7.0) < 1e-5);
        assert(std::abs(zVal.value() - 2.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR with non-boolean values" << RESET << std::endl;
    }
    // Test: Complex nested expression
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 1, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 1, 10);
        const auto& z = model.addVariable(CP::Variable::Type::INTEGER, "z", 3, 10);
        const auto& w = model.addVariable(CP::Variable::Type::INTEGER, "w", 2, 10);

        // Minimize (x + y) * (z - 2) / w
        // Subject to: x + y >= 5, z >= 3, w >= 2, all <= 10
        model.setObjective((x + y) * (z - 2.0) / w);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 1.0);
        model.addConstraint(x + y >= 5.0);
        model.addConstraint(x <= 10.0);
        model.addConstraint(y <= 10.0);
        model.addConstraint(z >= 3.0);
        model.addConstraint(z <= 10.0);
        model.addConstraint(w >= 2.0);
        model.addConstraint(w <= 10.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);
        auto zVal = solver.getSolution()->getVariableValue(z);
        auto wVal = solver.getSolution()->getVariableValue(w);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value() && wVal.has_value());

        // Verify optimal objective value
        auto objValue = solution->getObjectiveValue();
        assert(objValue.has_value());
        assert(std::abs(objValue.value() - 0.5) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Complex nested expression" << RESET << std::endl;
    }
    // Test: Logical OR with > comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint(y == 10.0);
        model.addConstraint(!flag || (x > y));
        model.addConstraint(flag == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);

        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() > yVal.value() - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR with > comparison" << RESET << std::endl;
    }
    // Test: Logical OR with < comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint(y == 10.0);
        model.addConstraint(!flag || (x < y));
        model.addConstraint(flag == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);

        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() < yVal.value() + 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR with < comparison" << RESET << std::endl;
    }
    // Test: Logical OR with != comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 49.0, 51.0);

        model.addConstraint(!flag || (x != 50.0));
        model.addConstraint(flag == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);

        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 50.0) > 1e-6);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR with != comparison" << RESET << std::endl;
    }
    // Test: Logical OR constraint (!a || b)
    {
        CP::Model model;
        const auto& a = model.addBinaryVariable("a");
        const auto& b = model.addBinaryVariable("b");
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);

        // Constraint: !a || (x >= y)
        // Meaning: if a is true, then x must be >= y
        model.addConstraint(!a || (x >= y));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();

        auto aVal = solver.getSolution()->getVariableValue(a);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);

        assert(aVal.has_value() && xVal.has_value() && yVal.has_value());

        // Verify the constraint: if a=1, then x >= y
        if (aVal.value() > 0.5) {
            assert(xVal.value() >= yVal.value() - 1e-6);
        }

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR constraint (!a || b)" << RESET << std::endl;
    }
    // Test: Logical OR constraint with comparison (!visit || (exit >= entry))
    {
        CP::Model model;
        const auto& visit = model.addBinaryVariable("visit");
        const auto& entry = model.addVariable(CP::Variable::Type::REAL, "entry", 0.0, 100.0);
        const auto& exit_var = model.addVariable(CP::Variable::Type::REAL, "exit", 0.0, 100.0);

        // Set entry to a specific value
        model.addConstraint(entry == 5.0);

        // Constraint: !visit || (exit >= entry)
        // Meaning: if visit is true, then exit must be >= entry
        model.addConstraint(!visit || (exit_var >= entry));

        // Force visit to be true to test the implication
        model.addConstraint(visit == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();

        auto visitVal = solver.getSolution()->getVariableValue(visit);
        auto entryVal = solver.getSolution()->getVariableValue(entry);
        auto exitVal = solver.getSolution()->getVariableValue(exit_var);

        assert(visitVal.has_value() && entryVal.has_value() && exitVal.has_value());

        // Since visit=1 and entry=5, exit must be >= 5
        assert(visitVal.value() > 0.5);
        assert(std::abs(entryVal.value() - 5.0) < 1e-5);
        assert(exitVal.value() >= 5.0 - 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Logical OR constraint with comparison (!visit || (exit >= entry))" << RESET << std::endl;
    }
    // Test: Deduced boolean variable in logical constraint
    {
        CP::Model model;
        const auto& visit = model.addVariable(CP::Variable::Type::BOOLEAN, "visit", 0.0, 1.0);
        const auto& tokenflow = model.addVariable(CP::Variable::Type::BOOLEAN, "tokenflow", visit);
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 10.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 10.0);

        // Constraint: !tokenflow || (x >= y)
        model.addConstraint(!tokenflow || (x >= y));

        // Set visit = 1, which should make tokenflow = 1
        model.addConstraint(visit == 1.0);

        // Set x = 5, y = 3, so x >= y is satisfied
        model.addConstraint(x == 5.0);
        model.addConstraint(y == 3.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto visitVal = solver.getSolution()->getVariableValue(visit);
        auto tokenflowVal = solution->evaluate(tokenflow);
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);

        assert(visitVal.has_value() && tokenflowVal.has_value() && xVal.has_value() && yVal.has_value());
        assert(std::abs(visitVal.value() - 1.0) < 1e-5);
        assert(std::abs(tokenflowVal.value() - 1.0) < 1e-5);
        assert(std::abs(xVal.value() - 5.0) < 1e-5);
        assert(std::abs(yVal.value() - 3.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Deduced boolean variable in logical constraint" << RESET << std::endl;
    }
    // Test: Deduced variable should fail when constraint is violated
    {
        CP::Model model;
        const auto& visit = model.addVariable(CP::Variable::Type::BOOLEAN, "visit", 0.0, 1.0);
        const auto& tokenflow = model.addVariable(CP::Variable::Type::BOOLEAN, "tokenflow", visit);
        const auto& exit_var = model.addVariable(CP::Variable::Type::REAL, "exit", 0.0, 10.0);
        const auto& value1 = model.addVariable(CP::Variable::Type::REAL, "value1", 0.0, 10.0);

        // Constraint: !tokenflow || (exit >= value1)
        model.addConstraint(!tokenflow || (exit_var >= value1));

        // Set visit = 1, which should make tokenflow = 1
        model.addConstraint(visit == 1.0);

        // Set exit = 0, value1 = 1, so exit >= value1 is violated
        // This should make the constraint fail
        model.addConstraint(exit_var == 0.0);
        model.addConstraint(value1 == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        // This problem should be infeasible
        assert(result.problem == CP::Solver::Result::PROBLEM::INFEASIBLE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Deduced variable enforces constraint correctly (infeasible)" << RESET << std::endl;
    }
    // Test: Deduced variable from IndexedVariable in constraint
    {
        CP::Model model;

        // Create indexed variables (like value_{Instance_1,Process_1},Instance)
        auto& processInstance = model.addIndexedVariables(CP::Variable::Type::REAL, "process_instance");
        model.addIndexedVariable(processInstance, 2.0, 2.0);  // processInstance[0] = 2

        // Create index variable (like data_index[Process_1]_{Instance_1,Activity_1,entry})
        const auto& dataIndex = model.addVariable(CP::Variable::Type::INTEGER, "data_index", 0.0, 0.0);

        // Create deduced variable (like value_{Instance_1,Activity_1,0},Instance)
        const auto& activityInstance = model.addVariable(CP::Variable::Type::REAL, "activity_instance", processInstance[dataIndex]);

        // This should constrain activity_instance = processInstance[0] = 2

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto dataIndexVal = solver.getSolution()->getVariableValue(dataIndex);
        auto activityInstanceVal = solution->evaluate(activityInstance);

        assert(dataIndexVal.has_value());
        assert(activityInstanceVal.has_value());
        assert(std::abs(dataIndexVal.value() - 0.0) < 1e-5);
        assert(std::abs(activityInstanceVal.value() - 2.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Deduced variable from IndexedVariable in constraint" << RESET << std::endl;
    }
    // Test: Expression::Operator::collection (wrapper)
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 5.0, 5.0);

        // Create collection expression wrapping x
        std::vector<CP::Operand> collectionOperands;
        collectionOperands.push_back(std::ref(x));
        CP::Expression collectionExpr(CP::Expression::Operator::collection, collectionOperands);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", collectionExpr);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto xVal = solver.getSolution()->getVariableValue(x);
        auto yVal = solver.getSolution()->getVariableValue(y);

        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(std::abs(xVal.value() - 5.0) < 1e-5);
        assert(std::abs(yVal.value() - 5.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Expression::Operator::collection" << RESET << std::endl;
    }
    // Test: Expression::Operator::at with collection lookup
    {
        CP::Model model;

        // Set up collection lookup
        std::vector<std::vector<double>> collections1 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections1](size_t key) -> const std::vector<double>& {
            return collections1[key];
        }, collections1.size());

        // Create collection(key)[index] using Expression::Operator::at directly
        const auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        std::vector<CP::Operand> collectionOperands;
        collectionOperands.push_back(std::ref(key));
        CP::Expression collectionExpr(CP::Expression::Operator::collection, collectionOperands);

        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 2.0, 2.0);
        std::vector<CP::Operand> atOperands;
        atOperands.push_back(collectionExpr);
        atOperands.push_back(std::ref(index));
        CP::Expression atExpr(CP::Expression::Operator::at, atOperands);

        const auto& result_var = model.addVariable(CP::Variable::Type::REAL, "result", atExpr);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();
        auto resultVal = solver.getSolution()->getVariableValue(result_var);

        assert(resultVal.has_value());
        // collection(0)[2] should be 20
        assert(std::abs(resultVal.value() - 20.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Expression::Operator::at with collection" << RESET << std::endl;
    }
    // Test: Collection lookup with count and at operators
    {
        // Mock collection registry
        std::vector<std::vector<double>> mockCollections = {
            {},                    // index 0: empty
            {10.0, 20.0, 30.0},   // index 1: 3 elements
            {5.0, 15.0}           // index 2: 2 elements
        };

        CP::Model model;

        // Set collection lookup on model
        model.setCollectionLookup([&mockCollections](size_t key) -> const std::vector<double>& {
            return mockCollections[key];
        }, mockCollections.size());

        // Use collection data to build constraints
        const auto& collection = model.getCollection(1);

        auto countExpr = CP::customOperator("count", collection[0], collection[1], collection[2]);
        auto& numElements = model.addVariable(CP::Variable::Type::INTEGER, "numElements", countExpr);

        auto atExpr = CP::customOperator("at", 2.0, collection[0], collection[1], collection[2]);

        auto& elementValue = model.addVariable(CP::Variable::Type::REAL, "elementValue", atExpr);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);

        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto solution = solver.getSolution();

        // Verify count returns 3
        auto numVal = solution->evaluate(numElements);
        assert(numVal.has_value());
        assert(std::abs(numVal.value() - 3.0) < 1e-5);

        // Verify at(2, ...) returns 20.0 (second element, 1-based indexing)
        auto elemVal = solution->evaluate(elementValue);
        assert(elemVal.has_value());
        assert(std::abs(elemVal.value() - 20.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Collection lookup with Model::setCollectionLookup" << RESET << std::endl;
    }

    // ==================== Collection Tests ====================

    // Test: count(collection(key)) with variable key
    {
        CP::Model model;
        std::vector<std::vector<double>> collections2 = {
            {10.0, 20.0, 30.0}, {40.0, 50.0}, {60.0, 70.0, 80.0, 90.0}
        };
        model.setCollectionLookup([&collections2](size_t key) -> const std::vector<double>& {
            return collections2[key];
        }, collections2.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 2.0);
        auto& variable = model.addVariable(CP::Variable::Type::INTEGER, "result", 0.0, 10.0);
        model.addConstraint(variable == CP::count(CP::Collection(key)));
        model.addConstraint(key == 0.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(variable).value() - 3.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: count(collection(key)) returns 3" << RESET << std::endl;
    }
    // Test: sum(collection(key))
    {
        CP::Model model;
        std::vector<std::vector<double>> collections3 = {{10.0, 20.0, 30.0}, {5.0, 15.0}};
        model.setCollectionLookup([&collections3](size_t key) -> const std::vector<double>& {
            return collections3[key];
        }, collections3.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
        auto& variable = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(variable == CP::sum(CP::Collection(key)));
        model.addConstraint(key == 1.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(variable).value() - 20.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: sum(collection(key)) returns 20" << RESET << std::endl;
    }
    // Test: avg(collection(key))
    {
        CP::Model model;
        std::vector<std::vector<double>> collections4 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections4](size_t key) -> const std::vector<double>& {
            return collections4[key];
        }, collections4.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 50.0);
        model.addConstraint(resultVar == CP::avg(CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 20.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: avg(collection(key)) returns 20" << RESET << std::endl;
    }
    // Test: max(collection(key))
    {
        CP::Model model;
        std::vector<std::vector<double>> collections5 = {{10.0, 50.0, 30.0}};
        model.setCollectionLookup([&collections5](size_t key) -> const std::vector<double>& {
            return collections5[key];
        }, collections5.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::max(CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 50.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: max(collection(key)) returns 50" << RESET << std::endl;
    }
    // Test: min(collection(key))
    {
        CP::Model model;
        std::vector<std::vector<double>> collections6 = {{30.0, 10.0, 50.0}};
        model.setCollectionLookup([&collections6](size_t key) -> const std::vector<double>& {
            return collections6[key];
        }, collections6.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::min(CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 10.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: min(collection(key)) returns 10" << RESET << std::endl;
    }
    // Test: element_of(constant, collection) - found
    {
        CP::Model model;
        std::vector<std::vector<double>> collections7 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections7](size_t key) -> const std::vector<double>& {
            return collections7[key];
        }, collections7.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::element_of(20.0, CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 1.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: element_of(20, collection) returns 1" << RESET << std::endl;
    }
    // Test: element_of(constant, collection) - not found
    {
        CP::Model model;
        std::vector<std::vector<double>> collections8 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections8](size_t key) -> const std::vector<double>& {
            return collections8[key];
        }, collections8.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::element_of(25.0, CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 0.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: element_of(25, collection) returns 0" << RESET << std::endl;
    }
    // Test: not_element_of(constant, collection) - found (returns 0)
    {
        CP::Model model;
        std::vector<std::vector<double>> collections9 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections9](size_t key) -> const std::vector<double>& {
            return collections9[key];
        }, collections9.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::not_element_of(20.0, CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 0.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: not_element_of(20, collection) returns 0" << RESET << std::endl;
    }
    // Test: not_element_of(constant, collection) - not found (returns 1)
    {
        CP::Model model;
        std::vector<std::vector<double>> collections10 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections10](size_t key) -> const std::vector<double>& {
            return collections10[key];
        }, collections10.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::not_element_of(25.0, CP::Collection(key)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 1.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: not_element_of(25, collection) returns 1" << RESET << std::endl;
    }
    // Test: Collection[constant_index]
    {
        CP::Model model;
        std::vector<std::vector<double>> collections11 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections11](size_t key) -> const std::vector<double>& {
            return collections11[key];
        }, collections11.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::Collection(key)[2.0]);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 20.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Collection[2] returns 20" << RESET << std::endl;
    }
    // Test: at with different collection keys
    {
        CP::Model model;
        std::vector<std::vector<double>> collections12 = {{10.0, 20.0, 30.0}, {40.0, 50.0, 60.0}};
        model.setCollectionLookup([&collections12](size_t key) -> const std::vector<double>& {
            return collections12[key];
        }, collections12.size());

        // Test with key=0
        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::Collection(key)[2.0]);

        CP::HexalySolver solver1(model);
        auto result1 = solver1.solve(5.0);
        assert(result1.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver1.getSolution()->getVariableValue(resultVar).value() - 20.0) < 1e-5);

        // Test with key=1
        CP::Model model2;
        model2.setCollectionLookup([&collections12](size_t key) -> const std::vector<double>& {
            return collections12[key];
        }, collections12.size());

        auto& key2 = model2.addVariable(CP::Variable::Type::INTEGER, "key", 1.0, 1.0);
        auto& result2 = model2.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model2.addConstraint(result2 == CP::Collection(key2)[2.0]);

        CP::HexalySolver solver2(model2);
        auto result2_solve = solver2.solve(5.0);
        assert(result2_solve.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver2.getSolution()->getVariableValue(result2).value() - 50.0) < 1e-5);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: at with different collection keys" << RESET << std::endl;
    }
    // Test: element_of(variable, collection) - found
    {
        CP::Model model;
        std::vector<std::vector<double>> collections13 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections13](size_t key) -> const std::vector<double>& {
            return collections13[key];
        }, collections13.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::element_of(value, CP::Collection(key)));
        model.addConstraint(value == 20.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 1.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: element_of(variable=20, collection) returns 1" << RESET << std::endl;
    }
    // Test: element_of(variable, collection) - not found
    {
        CP::Model model;
        std::vector<std::vector<double>> collections14 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections14](size_t key) -> const std::vector<double>& {
            return collections14[key];
        }, collections14.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::element_of(value, CP::Collection(key)));
        model.addConstraint(value == 25.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 0.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: element_of(variable=25, collection) returns 0" << RESET << std::endl;
    }
    // Test: element_of(variable, collection(variable_key)) - both variable
    {
        CP::Model model;
        std::vector<std::vector<double>> collections15 = {{10.0, 20.0, 30.0}, {40.0, 50.0, 60.0}};
        model.setCollectionLookup([&collections15](size_t key) -> const std::vector<double>& {
            return collections15[key];
        }, collections15.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
        auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 60.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::element_of(value, CP::Collection(key)));
        model.addConstraint(key == 1.0);
        model.addConstraint(value == 50.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 1.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: element_of(variable, collection(variable_key))" << RESET << std::endl;
    }
    // Test: not_element_of(variable, collection) - found (returns 0)
    {
        CP::Model model;
        std::vector<std::vector<double>> collections16 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections16](size_t key) -> const std::vector<double>& {
            return collections16[key];
        }, collections16.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::not_element_of(value, CP::Collection(key)));
        model.addConstraint(value == 20.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 0.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: not_element_of(variable=20, collection) returns 0" << RESET << std::endl;
    }
    // Test: not_element_of(variable, collection) - not found (returns 1)
    {
        CP::Model model;
        std::vector<std::vector<double>> collections17 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections17](size_t key) -> const std::vector<double>& {
            return collections17[key];
        }, collections17.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& value = model.addVariable(CP::Variable::Type::INTEGER, "value", 10.0, 40.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::BOOLEAN, "result", 0.0, 1.0);
        model.addConstraint(resultVar == CP::not_element_of(value, CP::Collection(key)));
        model.addConstraint(value == 35.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 1.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: not_element_of(variable=35, collection) returns 1" << RESET << std::endl;
    }
    // Test: Collection[variable_index]
    {
        CP::Model model;
        std::vector<std::vector<double>> collections18 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections18](size_t key) -> const std::vector<double>& {
            return collections18[key];
        }, collections18.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 0.0);
        auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 1.0, 3.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::Collection(key)[index]);
        model.addConstraint(index == 2.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 20.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Collection[variable_index=2] returns 20" << RESET << std::endl;
    }
    // Test: Collection(variable_key)[variable_index]
    {
        CP::Model model;
        std::vector<std::vector<double>> collections19 = {{10.0, 20.0, 30.0}, {40.0, 50.0, 60.0}};
        model.setCollectionLookup([&collections19](size_t key) -> const std::vector<double>& {
            return collections19[key];
        }, collections19.size());

        auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0.0, 1.0);
        auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 1.0, 3.0);
        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::Collection(key)[index]);
        model.addConstraint(key == 1.0);
        model.addConstraint(index == 3.0);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 60.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Collection(key=1)[index=3] returns 60" << RESET << std::endl;
    }
    // Test: count(Collection(constant_key))
    {
        CP::Model model;
        std::vector<std::vector<double>> collections20 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections20](size_t key) -> const std::vector<double>& {
            return collections20[key];
        }, collections20.size());

        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 10.0);
        model.addConstraint(resultVar == CP::count(CP::Collection(0.0)));

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 3.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: count(Collection(0)) returns 3" << RESET << std::endl;
    }
    // Test: Collection(constant_key)[constant_index]
    {
        CP::Model model;
        std::vector<std::vector<double>> collections21 = {{10.0, 20.0, 30.0}};
        model.setCollectionLookup([&collections21](size_t key) -> const std::vector<double>& {
            return collections21[key];
        }, collections21.size());

        auto& resultVar = model.addVariable(CP::Variable::Type::REAL, "result", 0.0, 100.0);
        model.addConstraint(resultVar == CP::Collection(0.0)[2.0]);

        CP::HexalySolver solver(model);
        auto result = solver.solve(5.0);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(std::abs(solver.getSolution()->getVariableValue(resultVar).value() - 20.0) < 1e-5);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Collection(0)[2] returns 20" << RESET << std::endl;
    }
    // Test: Iteration callback with stop() - knapsack problem
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const int n = 30;
        std::vector<std::reference_wrapper<const CP::Variable>> items;
        std::vector<double> weights = {23, 31, 29, 44, 53, 38, 63, 85, 89, 82,
                                        47, 52, 43, 67, 73, 59, 61, 77, 41, 37,
                                        26, 33, 48, 55, 62, 71, 79, 84, 91, 95};
        std::vector<double> values =  {92, 57, 49, 68, 60, 43, 67, 84, 87, 72,
                                        61, 55, 48, 72, 78, 64, 69, 82, 53, 47,
                                        89, 58, 51, 66, 75, 81, 70, 88, 94, 99};
        for (int i = 0; i < n; i++) {
            items.push_back(model.addVariable(CP::Variable::Type::INTEGER, "item" + std::to_string(i), 0.0, 1.0));
        }
        CP::Expression totalWeight = 0.0;
        CP::Expression totalValue = 0.0;
        for (int i = 0; i < n; i++) {
            totalWeight = totalWeight + items[i].get() * weights[i];
            totalValue = totalValue + items[i].get() * values[i];
        }
        model.addConstraint(totalWeight <= 400.0);
        model.setObjective(totalValue);

        CP::HexalySolver solver(model);
        bool callbackFired = false;
        solver.registerListener(CP::Solver::IterationListener([&]() {
            callbackFired = true;
            solver.stop();
        }));

        auto result = solver.solve();
        if (callbackFired) {
            assert(result.termination == CP::Solver::Result::TERMINATION::INTERRUPTED);
        }
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Iteration callback and stop()" << RESET << std::endl;
    }
    // Test: Solution callback
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0.0, 100.0);
        auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0.0, 100.0);
        model.addConstraint(x + y <= 100.0);
        model.setObjective(x + y);

        CP::HexalySolver solver(model);
        int solutionCount = 0;
        solver.registerListener(CP::Solver::SolutionListener([&](const CP::Solution& sol) {
            solutionCount++;
        }));

        auto result = solver.solve();
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        assert(solutionCount >= 1);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Solution callback" << RESET << std::endl;
    }
    // Test: Warmstart
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0.0, 100.0);
        auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0.0, 100.0);
        model.addConstraint(x + y <= 100.0);
        model.setObjective(x + y);

        auto initialSolution = std::make_shared<CP::Solution>(model);
        initialSolution->setVariableValue(x, 40.0);
        initialSolution->setVariableValue(y, 40.0);

        CP::HexalySolver solver(model);
        solver.setSolution(initialSolution);

        auto result = solver.solve(0.5);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto finalSol = solver.getSolution();
        double finalObj = finalSol->getVariableValue(x).value() + finalSol->getVariableValue(y).value();
        assert(finalObj >= 80.0 - 1e-5);  // At least as good as warmstart
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Warmstart" << RESET << std::endl;
    }
    // Test: Warmstart with indexed variables
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        auto& indexedVars = model.addIndexedVariables(CP::Variable::Type::INTEGER, "x");
        auto& x0 = model.addIndexedVariable(indexedVars, 0.0, 100.0);
        auto& x1 = model.addIndexedVariable(indexedVars, 0.0, 100.0);
        auto& x2 = model.addIndexedVariable(indexedVars, 0.0, 100.0);
        model.addConstraint(x0 + x1 + x2 <= 150.0);
        model.setObjective(x0 + x1 + x2);

        // Warmstart with a good solution: x0=40, x1=40, x2=40 (objective = 120)
        auto initialSolution = std::make_shared<CP::Solution>(model);
        initialSolution->setVariableValue(x0, 40.0);
        initialSolution->setVariableValue(x1, 40.0);
        initialSolution->setVariableValue(x2, 40.0);

        CP::HexalySolver solver(model);
        solver.setSolution(initialSolution);

        auto result = solver.solve(0.5);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto finalSol = solver.getSolution();
        double finalObj = finalSol->getVariableValue(x0).value()
                        + finalSol->getVariableValue(x1).value()
                        + finalSol->getVariableValue(x2).value();
        assert(finalObj >= 120.0 - 1e-5);  // At least as good as warmstart
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Warmstart with indexed variables" << RESET << std::endl;
    }
    // Test: Warmstart with sequence variables
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        auto& seq = model.addSequence("seq", 4);
        model.setObjective(seq.variables[0]);  // Minimize value at position 0

        // Warmstart: [2,1,3,4] gives objective 2
        auto initialSolution = std::make_shared<CP::Solution>(model);
        initialSolution->setSequenceValues(seq, std::vector<int>{2, 1, 3, 4});

        CP::HexalySolver solver(model);
        solver.setSolution(initialSolution);

        auto result = solver.solve(0.5);
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        auto finalSol = solver.getSolution();
        double finalObj = finalSol->getVariableValue(seq.variables[0]).value();
        assert(finalObj <= 2.0 + 1e-5);  // At least as good as warmstart
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Warmstart with sequence variables" << RESET << std::endl;
    }

    // Test: Warmstart with fixed boolean variable (should not crash)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        // Fixed boolean: lb == ub, becomes constant in Hexaly (not a decision)
        const auto& fixed = model.addVariable(CP::Variable::Type::BOOLEAN, "fixed", 1, 1);
        model.setObjective(x);

        // Warmstart includes the fixed variable
        auto initialSolution = std::make_shared<CP::Solution>(model);
        initialSolution->setVariableValue(x, 5.0);
        initialSolution->setVariableValue(fixed, 1.0);

        CP::HexalySolver solver(model);
        solver.setSolution(initialSolution);

        auto result = solver.solve(0.5);
        // Should not fail with "Only decisions can be set."
        if (result.status == CP::Solver::Result::SOLUTION::NONE) {
            std::cerr << "ERROR: " << result.info << std::endl;
        }
        assert(result.status != CP::Solver::Result::SOLUTION::NONE);
        std::cout << GREEN << "Test " << ++testNum << " PASSED: Warmstart with fixed boolean variable" << RESET << std::endl;
    }

    // Test: Fix variable changes solution
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        model.setObjective(x);

        CP::HexalySolver solver(model);

        // Stop on first solution
        solver.registerListener(CP::Solver::SolutionListener([&](const CP::Solution&) {
            solver.stop();
        }));

        // First solve - optimal should be 0
        auto result1 = solver.solve();
        assert(result1.status != CP::Solver::Result::SOLUTION::NONE);
        assert(solver.getSolution()->getVariableValue(x).value() == 0.0);

        // Fix x to 5
        solver.fix(x, 5.0);

        // Second solve - should be 5
        auto result2 = solver.solve();
        assert(result2.status != CP::Solver::Result::SOLUTION::NONE);
        assert(solver.getSolution()->getVariableValue(x).value() == 5.0);

        // Unfix and solve again - should return to 0
        solver.unfix();
        auto result3 = solver.solve();
        assert(result3.status != CP::Solver::Result::SOLUTION::NONE);
        assert(solver.getSolution()->getVariableValue(x).value() == 0.0);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Fix variable changes solution" << RESET << std::endl;
    }

    // Test: Fix sequence changes solution
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& seq = model.addSequence("seq", 3);

        // Objective: minimize sum of (position * value) where positions are 1,2,3
        // Optimal: seq[0]=3, seq[1]=2, seq[2]=1 gives 1*3 + 2*2 + 3*1 = 10
        auto obj = 1.0 * seq.variables[0] + 2.0 * seq.variables[1] + 3.0 * seq.variables[2];
        model.setObjective(obj);

        CP::HexalySolver solver(model);

        // Stop on first solution
        solver.registerListener(CP::Solver::SolutionListener([&](const CP::Solution&) {
            solver.stop();
        }));

        // First solve - find any solution
        auto result1 = solver.solve();
        assert(result1.status != CP::Solver::Result::SOLUTION::NONE);

        // Fix sequence to [1,2,3]
        solver.fix(seq, {1, 2, 3});

        // Second solve - should be [1,2,3]
        auto result2 = solver.solve();
        assert(result2.status != CP::Solver::Result::SOLUTION::NONE);
        auto sol2 = solver.getSolution();
        assert(sol2->getVariableValue(seq.variables[0]).value() == 1.0);
        assert(sol2->getVariableValue(seq.variables[1]).value() == 2.0);
        assert(sol2->getVariableValue(seq.variables[2]).value() == 3.0);

        // Unfix and solve again - should find a solution
        solver.unfix();
        auto result3 = solver.solve();
        assert(result3.status != CP::Solver::Result::SOLUTION::NONE);

        std::cout << GREEN << "Test " << ++testNum << " PASSED: Fix sequence changes solution" << RESET << std::endl;
    }

    std::cout << "\n" << GREEN << "All " << testNum << " Hexaly adapter tests PASSED" << RESET << std::endl;
    return 0;
}
