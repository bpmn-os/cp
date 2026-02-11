#include "cp.h"
#include "scip/scip_adapter.h"
#include <scip/scip.h>
#include <iostream>
#include <cassert>
#include <cmath>

#define GREEN "\033[32m"
#define RESET "\033[0m"

int main() {
    // Test 1: Single integer variable
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();
        const auto& varMap = solver.getVariableMap();

        assert(SCIPgetNVars(scip) == 1);
        assert(solver.getName() == "SCIP");

        auto it = varMap.find(&x);
        assert(it != varMap.end());
        assert(SCIPvarGetType(it->second) == SCIP_VARTYPE_INTEGER);

        std::cout << GREEN << "Test 1 PASSED: Single integer variable" << RESET << std::endl;
    }

    // Test 2: Multiple variable types
    {
        CP::Model model;
        const auto& b = model.addBinaryVariable("b");
        const auto& i = model.addIntegerVariable("i");
        const auto& r = model.addRealVariable("r");

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();
        const auto& varMap = solver.getVariableMap();

        assert(SCIPgetNVars(scip) == 3);
        assert(varMap.size() == 3);

        auto itB = varMap.find(&b);
        auto itI = varMap.find(&i);
        auto itR = varMap.find(&r);
        assert(itB != varMap.end());
        assert(itI != varMap.end());
        assert(itR != varMap.end());

        assert(SCIPvarGetType(itB->second) == SCIP_VARTYPE_BINARY);
        assert(SCIPvarGetType(itI->second) == SCIP_VARTYPE_INTEGER);
        assert(SCIPvarGetType(itR->second) == SCIP_VARTYPE_CONTINUOUS);

        std::cout << GREEN << "Test 2 PASSED: Multiple variable types" << RESET << std::endl;
    }

    // Test 3: Indexed variables
    {
        CP::Model model;
        auto& vars = model.addIndexedVariables(CP::Variable::Type::INTEGER, "x");
        vars.emplace_back(0, 10);
        vars.emplace_back(5, 15);
        vars.emplace_back(-5, 5);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();
        const auto& varMap = solver.getVariableMap();

        assert(SCIPgetNVars(scip) == 3);

        auto it0 = varMap.find(&vars[0]);
        auto it1 = varMap.find(&vars[1]);
        auto it2 = varMap.find(&vars[2]);
        assert(it0 != varMap.end());
        assert(it1 != varMap.end());
        assert(it2 != varMap.end());

        assert(SCIPvarGetType(it0->second) == SCIP_VARTYPE_INTEGER);
        assert(SCIPvarGetType(it1->second) == SCIP_VARTYPE_INTEGER);
        assert(SCIPvarGetType(it2->second) == SCIP_VARTYPE_INTEGER);

        assert(SCIPvarGetLbGlobal(it0->second) == 0.0);
        assert(SCIPvarGetUbGlobal(it0->second) == 10.0);
        assert(SCIPvarGetLbGlobal(it1->second) == 5.0);
        assert(SCIPvarGetUbGlobal(it1->second) == 15.0);
        assert(SCIPvarGetLbGlobal(it2->second) == -5.0);
        assert(SCIPvarGetUbGlobal(it2->second) == 5.0);

        std::cout << GREEN << "Test 3 PASSED: Indexed variables" << RESET << std::endl;
    }

    // Test 4: Sequence with alldifferent
    {
        CP::Model model;
        model.addSequence("seq", 5);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        // Sequence creates 5 variables + 5×5=25 binary variables for alldifferent = 30 total
        assert(SCIPgetNVars(scip) == 30);
        std::cout << GREEN << "Test 4 PASSED: Sequence with alldifferent" << RESET << std::endl;
    }

    // Test 5: Minimize objective
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        model.setObjective(x);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetObjsense(scip) == SCIP_OBJSENSE_MINIMIZE);
        // With expression-based objectives, there's an auxiliary variable
        assert(SCIPgetNVars(scip) >= 1);

        std::cout << GREEN << "Test 5 PASSED: Minimize objective" << RESET << std::endl;
    }

    // Test 6: Maximize objective
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addIntegerVariable("x");
        model.setObjective(x);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetObjsense(scip) == SCIP_OBJSENSE_MAXIMIZE);
        assert(SCIPgetNVars(scip) >= 1);

        std::cout << GREEN << "Test 6 PASSED: Maximize objective" << RESET << std::endl;
    }

    // Test 7: Linear objective with coefficients (2*x + 3*y)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.setObjective(2.0 * x + 3.0 * y);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNVars(scip) >= 2);
        assert(SCIPgetObjsense(scip) == SCIP_OBJSENSE_MINIMIZE);

        std::cout << GREEN << "Test 7 PASSED: Linear objective with coefficients" << RESET << std::endl;
    }

    // Test 8: Linear objective with subtraction (x - 2*y)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.setObjective(x - 2.0 * y);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetObjsense(scip) == SCIP_OBJSENSE_MAXIMIZE);
        assert(SCIPgetNVars(scip) >= 2);

        std::cout << GREEN << "Test 8 PASSED: Linear objective with subtraction" << RESET << std::endl;
    }

    // Test 9: Negated variable (-x)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        model.setObjective(-x);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNVars(scip) >= 1);

        std::cout << GREEN << "Test 9 PASSED: Negated variable objective" << RESET << std::endl;
    }

    // Test 10: Feasibility (no objective)
    {
        CP::Model model(CP::Model::ObjectiveSense::FEASIBLE);
        model.addIntegerVariable("x");
        model.addIntegerVariable("y");

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNVars(scip) == 2);
        std::cout << GREEN << "Test 10 PASSED: Feasibility problem" << RESET << std::endl;
    }

    // Test 11: Simple equality constraint (x == 5)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        model.addConstraint(x == 5.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 11 PASSED: Equality constraint (feasible)" << RESET << std::endl;
    }

    // Test 12: Equality constraint (infeasible: x == 5 with x=6)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        // Add constraint x == 5
        model.addConstraint(x == 5.0);

        // Fix x to 6 - should be infeasible
        model.addConstraint(x >= 6.0);
        model.addConstraint(x <= 6.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // Should have no solution (6 == 5 is false)
        assert(!result.has_value());

        std::cout << GREEN << "Test 12 PASSED: Equality constraint (infeasible)" << RESET << std::endl;
    }

    // Test 13: Simple inequality (x <= 10)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        model.addConstraint(x <= 10.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 13 PASSED: Less-or-equal constraint (feasible)" << RESET << std::endl;
    }

    // Test 14: Less-or-equal constraint (infeasible: x <= 5 with x=6)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        // Add constraint x <= 5
        model.addConstraint(x <= 5.0);

        // Fix x to 6 - should be infeasible
        model.addConstraint(x >= 6.0);
        model.addConstraint(x <= 6.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // Should have no solution (6 <= 5 is false)
        assert(!result.has_value());

        std::cout << GREEN << "Test 14 PASSED: Less-or-equal constraint (infeasible)" << RESET << std::endl;
    }

    // Test 15: Greater-or-equal (x >= 0)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        model.addConstraint(x >= 0.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 15 PASSED: Greater-or-equal constraint (feasible)" << RESET << std::endl;
    }

    // Test 16: Greater-or-equal constraint (infeasible: x >= 5 with x=4)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        // Add constraint x >= 5
        model.addConstraint(x >= 5.0);

        // Fix x to 4 - should be infeasible
        model.addConstraint(x >= 4.0);
        model.addConstraint(x <= 4.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // Should have no solution (4 >= 5 is false)
        assert(!result.has_value());

        std::cout << GREEN << "Test 16 PASSED: Greater-or-equal constraint (infeasible)" << RESET << std::endl;
    }

    // Test 17: Linear constraint (2*x + 3*y <= 15)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.addConstraint(2.0 * x + 3.0 * y <= 15.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNVars(scip) == 2);
        assert(SCIPgetNConss(scip) == 1);

        SCIP_CONS** conss = SCIPgetConss(scip);
        assert(conss != nullptr);

        std::cout << GREEN << "Test 17 PASSED: Linear constraint" << RESET << std::endl;
    }

    // Test 18: Equality with two variables (x + y == 10)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.addConstraint(x + y == 10.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 18 PASSED: Two-variable equality" << RESET << std::endl;
    }

    // Test 19: Multiple constraints
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);
        model.addConstraint(x + y <= 100.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 3);
        std::cout << GREEN << "Test 19 PASSED: Multiple constraints" << RESET << std::endl;
    }

    // Test 20: Constraint with subtraction (x - y >= 5)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.addConstraint(x - y >= 5.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 20 PASSED: Subtraction constraint" << RESET << std::endl;
    }

    // Test 21: Constraint with constant on left (10 <= x + y)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.addConstraint(10.0 <= x + y);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 21 PASSED: Constant on left side" << RESET << std::endl;
    }

    // Test 22: Solve simple problem (minimize x, x >= 5)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        model.setObjective(x);
        model.addConstraint(x >= 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() == 5.0);

        std::cout << GREEN << "Test 22 PASSED: Minimize with lower bound" << RESET << std::endl;
    }

    // Test 23: Maximize problem (maximize x, x <= 10)
    {
        CP::Model model(CP::Model::ObjectiveSense::MAXIMIZE);
        const auto& x = model.addIntegerVariable("x");
        model.setObjective(x);
        model.addConstraint(x <= 10.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() == 10.0);

        std::cout << GREEN << "Test 23 PASSED: Maximize with upper bound" << RESET << std::endl;
    }

    // Test 24: Two variables (minimize x + y, x >= 2, y >= 3)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.setObjective(x + y);
        model.addConstraint(x >= 2.0);
        model.addConstraint(y >= 3.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(xVal.value() == 2.0);
        assert(yVal.value() == 3.0);

        std::cout << GREEN << "Test 24 PASSED: Two variables minimize" << RESET << std::endl;
    }

    // Test 25: Linear programming (minimize 2*x + 3*y, x + y >= 10, x >= 0, y >= 0)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.setObjective(2.0 * x + 3.0 * y);
        model.addConstraint(x + y >= 10.0);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        // Optimal: x=10, y=0 (objective = 20)
        assert(xVal.value() == 10.0);
        assert(yVal.value() == 0.0);

        std::cout << GREEN << "Test 25 PASSED: Linear programming" << RESET << std::endl;
    }

    // Test 26: Equality constraint (x + y == 7, minimize x, x >= 0, y >= 0)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.setObjective(x);
        model.addConstraint(x + y == 7.0);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        // Optimal: x=0, y=7
        assert(xVal.value() == 0.0);
        assert(yVal.value() == 7.0);

        std::cout << GREEN << "Test 26 PASSED: Equality constraint" << RESET << std::endl;
    }

    // Test 27: Non-linear constraint (x * y <= 10)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.addConstraint(x * y <= 10.0);

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();

        assert(SCIPgetNConss(scip) == 1);
        std::cout << GREEN << "Test 27 PASSED: Non-linear constraint" << RESET << std::endl;
    }

    // Test 28: Solve non-linear (minimize x+y, x*y >= 12, x >= 1, y >= 1)
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        model.setObjective(x + y);
        model.addConstraint(x * y >= 12.0);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        // Optimal should be around x=3, y=4 or x=4, y=3 (both give 7)
        double product = xVal.value() * yVal.value();
        assert(product >= 12.0);
        assert(xVal.value() + yVal.value() <= 8.0); // Should be close to optimal

        std::cout << GREEN << "Test 28 PASSED: Solve non-linear problem" << RESET << std::endl;
    }

    // Test 29: Logical NOT constraint (!x == 0 => x must be 1)
    {
        CP::Model model;
        const auto& x = model.addBinaryVariable("x");
        model.addConstraint(!x == 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() == 1.0);

        std::cout << GREEN << "Test 29 PASSED: Logical NOT" << RESET << std::endl;
    }

    // Test 30: Logical AND constraint (x && y == 1)
    {
        CP::Model model;
        const auto& x = model.addBinaryVariable("x");
        const auto& y = model.addBinaryVariable("y");
        model.addConstraint((x && y) == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(xVal.value() == 1.0);
        assert(yVal.value() == 1.0);

        std::cout << GREEN << "Test 30 PASSED: Logical AND" << RESET << std::endl;
    }

    // Test 31: Logical OR constraint (x || y == 1, x == 0)
    {
        CP::Model model;
        const auto& x = model.addBinaryVariable("x");
        const auto& y = model.addBinaryVariable("y");
        model.addConstraint((x || y) == 1.0);
        model.addConstraint(x == 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(xVal.value() == 0.0);
        assert(yVal.value() == 1.0);

        std::cout << GREEN << "Test 31 PASSED: Logical OR" << RESET << std::endl;
    }

    // Test 32: Custom operator sum
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        const auto& z = model.addIntegerVariable("z");

        // Use CP::customOperator to create sum
        auto sumExpr = CP::customOperator("sum", x, y, z);
        model.setObjective(sumExpr);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 2.0);
        model.addConstraint(z >= 3.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        auto zVal = solution.getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());
        assert(xVal.value() == 1.0);
        assert(yVal.value() == 2.0);
        assert(zVal.value() == 3.0);

        std::cout << GREEN << "Test 32 PASSED: Custom operator sum" << RESET << std::endl;
    }

    // Test 33: Custom operator avg
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        const auto& z = model.addIntegerVariable("z");

        // Minimize avg(x, y, z) = (x + y + z) / 3
        auto avgExpr = CP::customOperator("avg", x, y, z);
        model.setObjective(avgExpr);
        model.addConstraint(x >= 1.0);
        model.addConstraint(y >= 2.0);
        model.addConstraint(z >= 3.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        auto zVal = solution.getVariableValue(z);
        assert(xVal.has_value() && yVal.has_value() && zVal.has_value());

        // Optimal: x=1, y=2, z=3, avg = 6/3 = 2
        assert(xVal.value() == 1.0);
        assert(yVal.value() == 2.0);
        assert(zVal.value() == 3.0);

        std::cout << GREEN << "Test 33 PASSED: Custom operator avg" << RESET << std::endl;
    }

    // Test 34: Custom operator pow (x^2 == 16)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        auto powExpr = CP::customOperator("pow", x, 2.0);
        model.addConstraint(powExpr == 16.0);
        model.addConstraint(x >= 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() == 4.0);

        std::cout << GREEN << "Test 34 PASSED: Custom operator pow" << RESET << std::endl;
    }

    // Test 35: Custom operator min
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        auto minExpr = CP::customOperator("min", x, y);
        model.setObjective(minExpr);
        model.addConstraint(x >= 5.0);
        model.addConstraint(y >= 3.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());

        // Objective should be min(x,y) = 3
        double minVal = std::min(xVal.value(), yVal.value());
        assert(minVal == 3.0);

        std::cout << GREEN << "Test 35 PASSED: Custom operator min" << RESET << std::endl;
    }

    // Test 36: Custom operator max
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        const auto& z = model.addIntegerVariable("z");

        auto maxExpr = CP::customOperator("max", x, y, z);
        model.setObjective(maxExpr);
        model.addConstraint(x >= 10.0);
        model.addConstraint(y >= 7.0);
        model.addConstraint(z >= 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        auto zVal = solution.getVariableValue(z);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(zVal.has_value());

        // Objective should minimize max(x,y,z) with x>=10, y>=7, z>=5, so max should be 10
        double maxVal = std::max({xVal.value(), yVal.value(), zVal.value()});
        assert(maxVal == 10.0);

        std::cout << GREEN << "Test 36 PASSED: Custom operator max" << RESET << std::endl;
    }

    // Test 37: Custom operator n_ary_if
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& selector = model.addBinaryVariable("selector");

        // if selector then x = 10 else x = 5
        CP::Cases cases = {
            {selector, 10.0}
        };
        auto ifExpr = CP::n_ary_if(cases, 5.0);
        model.addConstraint(x == ifExpr);
        model.setObjective(x);

        // Constrain selector to ensure deterministic test
        model.addConstraint(selector == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto selectorVal = solution.getVariableValue(selector);
        assert(xVal.has_value());
        assert(selectorVal.has_value());

        // selector=1, so x should be 10
        assert(selectorVal.value() == 1.0);
        assert(xVal.value() == 10.0);

        std::cout << GREEN << "Test 37 PASSED: Custom operator n_ary_if" << RESET << std::endl;
    }

    // Test 38: Custom operator if_then_else
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& condition = model.addBinaryVariable("condition");

        // if condition then 20 else 8
        auto ifExpr = CP::if_then_else(condition, 20.0, 8.0);
        model.addConstraint(x == ifExpr);
        model.setObjective(x);

        // Constrain condition to 0 to get deterministic result
        model.addConstraint(condition == 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto conditionVal = solution.getVariableValue(condition);
        assert(xVal.has_value());
        assert(conditionVal.has_value());

        // condition=0, so x should be 8
        assert(conditionVal.value() == 0.0);
        assert(xVal.value() == 8.0);

        std::cout << GREEN << "Test 38 PASSED: Custom operator if_then_else" << RESET << std::endl;
    }

    // Test 39: Sequence with alldifferent constraint
    {
        CP::Model model;
        const auto& seq = model.addSequence("perm", 4);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto seqVals = solution.getSequenceValues(seq);
        assert(seqVals.has_value());

        // Check that all values are different and in range [1, 4]
        std::vector<bool> seen(5, false);
        for (auto val : seqVals.value()) {
            assert(val >= 1.0 && val <= 4.0);
            int intVal = static_cast<int>(val);
            assert(!seen[intVal]); // Not seen before
            seen[intVal] = true;
        }

        std::cout << GREEN << "Test 39 PASSED: Sequence with alldifferent" << RESET << std::endl;
    }

    // Test 40: Indexed variables (element constraint)
    {
        CP::Model model;
        auto& arr = model.addIndexedVariables(CP::Variable::Type::INTEGER, "arr");
        arr.emplace_back(0, 10);  // arr[0]
        arr.emplace_back(0, 10);  // arr[1]
        arr.emplace_back(0, 10);  // arr[2]

        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 0, 2);
        const auto& result = model.addIntegerVariable("result");

        // result = arr[index]
        model.addConstraint(result == arr[index]);

        // Set specific values for array elements
        model.addConstraint(arr[0] == 5.0);
        model.addConstraint(arr[1] == 7.0);
        model.addConstraint(arr[2] == 3.0);

        // Fix index to 1
        model.addConstraint(index == 1.0);

        CP::SCIPSolver solver(model);
        auto result_solve = solver.solve(model);

        assert(result_solve.has_value());
        auto& solution = result_solve.value();
        auto indexVal = solution.getVariableValue(index);
        auto resultVal = solution.getVariableValue(result);
        assert(indexVal.has_value());
        assert(resultVal.has_value());

        // index=1, so result should be arr[1]=7
        assert(indexVal.value() == 1.0);
        assert(resultVal.value() == 7.0);

        std::cout << GREEN << "Test 40 PASSED: Indexed variables (element constraint)" << RESET << std::endl;
    }

    // Test 41: Custom operator at
    {
        CP::Model model;
        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 0, 2);
        const auto& result = model.addIntegerVariable("result");

        // result = at(index, 10, 20, 30) - pick from inline values
        auto atExpr = CP::customOperator("at", index, 10.0, 20.0, 30.0);
        model.addConstraint(result == atExpr);

        // Fix index to 1
        model.addConstraint(index == 1.0);

        CP::SCIPSolver solver(model);
        auto result_solve = solver.solve(model);

        assert(result_solve.has_value());
        auto& solution = result_solve.value();
        auto indexVal = solution.getVariableValue(index);
        auto resultVal = solution.getVariableValue(result);
        assert(indexVal.has_value());
        assert(resultVal.has_value());

        // index=1, so result should be 20 (second value, 0-indexed)
        assert(indexVal.value() == 1.0);
        assert(resultVal.value() == 20.0);

        std::cout << GREEN << "Test 41 PASSED: Custom operator at" << RESET << std::endl;
    }

    // Test 42: Not-equal constraint
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        // Add constraint x != y
        model.addConstraint(x != y);

        // Fix x to 5
        model.addConstraint(x >= 5.0);
        model.addConstraint(x <= 5.0);

        // Constrain y to a range that includes 5
        model.addConstraint(y >= 3.0);
        model.addConstraint(y <= 7.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());

        // x should be 5, y should be anything but 5
        assert(xVal.value() == 5.0);
        assert(yVal.value() != 5.0);

        std::cout << GREEN << "Test 42 PASSED: Not-equal constraint (feasible)" << RESET << std::endl;
    }

    // Test 43: Not-equal constraint (infeasible: x != 5 with x=5)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        // Add constraint x != 5
        model.addConstraint(x != 5.0);

        // Fix x to 5 - should be infeasible
        model.addConstraint(x >= 5.0);
        model.addConstraint(x <= 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // Should have no solution (5 != 5 is false)
        assert(!result.has_value());

        std::cout << GREEN << "Test 43 PASSED: Not-equal constraint (infeasible)" << RESET << std::endl;
    }

    // Test 44: Less-than constraint (infeasible: x < y with x=5, y=5)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        // Add constraint x < y
        model.addConstraint(x < y);

        // Fix both to 5 - should be infeasible
        model.addConstraint(x >= 5.0);
        model.addConstraint(x <= 5.0);
        model.addConstraint(y >= 5.0);
        model.addConstraint(y <= 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // Should have no solution (5 < 5 is false)
        assert(!result.has_value());

        std::cout << GREEN << "Test 44 PASSED: Less-than constraint (infeasible)" << RESET << std::endl;
    }

    // Test 45: Less-than constraint (feasible: x < y with x=5, y=6)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        // Add constraint x < y
        model.addConstraint(x < y);

        // Fix x=5, y=6 - should be feasible
        model.addConstraint(x >= 5.0);
        model.addConstraint(x <= 5.0);
        model.addConstraint(y >= 6.0);
        model.addConstraint(y <= 6.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(xVal.value() == 5.0);
        assert(yVal.value() == 6.0);

        std::cout << GREEN << "Test 45 PASSED: Less-than constraint (feasible)" << RESET << std::endl;
    }

    // Test 46: Greater-than constraint (infeasible: x > y with x=5, y=5)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        // Add constraint x > y
        model.addConstraint(x > y);

        // Fix both to 5 - should be infeasible
        model.addConstraint(x >= 5.0);
        model.addConstraint(x <= 5.0);
        model.addConstraint(y >= 5.0);
        model.addConstraint(y <= 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // Should have no solution (5 > 5 is false)
        assert(!result.has_value());

        std::cout << GREEN << "Test 46 PASSED: Greater-than constraint (infeasible)" << RESET << std::endl;
    }

    // Test 47: Greater-than constraint (feasible: x > y with x=6, y=5)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        // Add constraint x > y
        model.addConstraint(x > y);

        // Fix x=6, y=5 - should be feasible
        model.addConstraint(x >= 6.0);
        model.addConstraint(x <= 6.0);
        model.addConstraint(y >= 5.0);
        model.addConstraint(y <= 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(xVal.value() == 6.0);
        assert(yVal.value() == 5.0);

        std::cout << GREEN << "Test 47 PASSED: Greater-than constraint (feasible)" << RESET << std::endl;
    }

    // Test 48: Division operator
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");

        // Minimize x/y subject to x >= 12, y >= 1, y <= 3, x <= 20
        model.setObjective(x / y);
        model.addConstraint(x >= 12.0);
        model.addConstraint(x <= 20.0);
        model.addConstraint(y >= 1.0);
        model.addConstraint(y <= 3.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());

        // Optimal: x=12, y=3, x/y = 4 (minimize ratio)
        assert(xVal.value() == 12.0);
        assert(yVal.value() == 3.0);

        std::cout << GREEN << "Test 48 PASSED: Division operator" << RESET << std::endl;
    }

    // Test 49: Logical NOT with non-boolean value (!5 should be 0)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        // !x == 0 when x != 0
        model.addConstraint(x == 5.0);
        model.addConstraint((!x) == 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        assert(xVal.has_value());
        assert(xVal.value() == 5.0);

        std::cout << GREEN << "Test 49 PASSED: Logical NOT with non-boolean value" << RESET << std::endl;
    }

    // Test 50: Logical AND with non-boolean values (3 && 5 should be 1)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        const auto& z = model.addIntegerVariable("z");

        // Set x=3, y=5, then (x && y) should equal 1
        model.addConstraint(x == 3.0);
        model.addConstraint(y == 5.0);
        model.addConstraint((x && y) == 1.0);
        model.addConstraint(z == 7.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        auto zVal = solution.getVariableValue(z);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(zVal.has_value());
        assert(xVal.value() == 3.0);
        assert(yVal.value() == 5.0);
        assert(zVal.value() == 7.0);

        std::cout << GREEN << "Test 50 PASSED: Logical AND with non-boolean values" << RESET << std::endl;
    }

    // Test 51: Logical OR with non-boolean values (0 || 7 should be 1)
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        const auto& z = model.addIntegerVariable("z");

        // Set x=0, y=7, then (x || y) should equal 1
        model.addConstraint(x == 0.0);
        model.addConstraint(y == 7.0);
        model.addConstraint((x || y) == 1.0);
        model.addConstraint(z == 2.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        auto zVal = solution.getVariableValue(z);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(zVal.has_value());
        assert(xVal.value() == 0.0);
        assert(yVal.value() == 7.0);
        assert(zVal.value() == 2.0);

        std::cout << GREEN << "Test 51 PASSED: Logical OR with non-boolean values" << RESET << std::endl;
    }

    // Test 52: Complex nested expression
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");
        const auto& y = model.addIntegerVariable("y");
        const auto& z = model.addIntegerVariable("z");
        const auto& w = model.addIntegerVariable("w");

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

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        auto zVal = solution.getVariableValue(z);
        auto wVal = solution.getVariableValue(w);
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(zVal.has_value());
        assert(wVal.has_value());

        // Verify optimal objective value
        auto objValue = solution.getObjectiveValue();
        assert(objValue.has_value());
        assert(std::abs(objValue.value() - 0.5) < 1e-5);

        std::cout << GREEN << "Test 52 PASSED: Complex nested expression" << RESET << std::endl;
    }

    // Test 53: Unbounded variables
    {
        CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
        const auto& x = model.addIntegerVariable("x");  // Unbounded
        const auto& y = model.addIntegerVariable("y");  // Unbounded

        // Minimize x + y subject to x + y >= 10, x >= 0, y >= 0
        model.setObjective(x + y);
        model.addConstraint(x + y >= 10.0);
        model.addConstraint(x >= 0.0);
        model.addConstraint(y >= 0.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);
        assert(xVal.has_value());
        assert(yVal.has_value());

        // Optimal: x + y = 10 (any combination where x >= 0, y >= 0, x + y = 10)
        assert(xVal.value() + yVal.value() == 10.0);
        assert(xVal.value() >= 0.0);
        assert(yVal.value() >= 0.0);

        std::cout << GREEN << "Test 53 PASSED: Unbounded variables" << RESET << std::endl;
    }

    // Test 54: Logical OR constraint (!a || b)
    {
        CP::Model model;
        const auto& a = model.addBinaryVariable("a");
        const auto& b = model.addBinaryVariable("b");
        const auto& x = model.addVariable(CP::Variable::Type::INTEGER, "x", 0, 10);
        const auto& y = model.addVariable(CP::Variable::Type::INTEGER, "y", 0, 10);

        // Constraint: !a || (x >= y)
        // Meaning: if a is true, then x must be >= y
        model.addConstraint(!a || (x >= y));

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();

        auto aVal = solution.getVariableValue(a);
        auto bVal = solution.getVariableValue(b);
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);

        assert(aVal.has_value());
        assert(xVal.has_value());
        assert(yVal.has_value());

        // Verify the constraint: if a=1, then x >= y
        if (aVal.value() > 0.5) {  // a is true
            assert(xVal.value() >= yVal.value() - 1e-6);
        }

        std::cout << GREEN << "Test 54 PASSED: Logical OR constraint (!a || b)" << RESET << std::endl;
    }

    // Test 55: Logical OR constraint with comparison (!visit || (exit >= entry))
    {
        CP::Model model;
        const auto& visit = model.addBinaryVariable("visit");
        const auto& entry = model.addVariable(CP::Variable::Type::REAL, "entry", 0.0, 100.0);
        const auto& exit = model.addVariable(CP::Variable::Type::REAL, "exit", 0.0, 100.0);

        // Set entry to a specific value
        model.addConstraint(entry == 5.0);

        // Constraint: !visit || (exit >= entry)
        // Meaning: if visit is true, then exit must be >= entry
        model.addConstraint(!visit || (exit >= entry));

        // Force visit to be true to test the implication
        model.addConstraint(visit == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();

        auto visitVal = solution.getVariableValue(visit);
        auto entryVal = solution.getVariableValue(entry);
        auto exitVal = solution.getVariableValue(exit);

        assert(visitVal.has_value());
        assert(entryVal.has_value());
        assert(exitVal.has_value());

        // Since visit=1 and entry=5, exit must be >= 5
        assert(visitVal.value() > 0.5);  // visit is true
        assert(std::abs(entryVal.value() - 5.0) < 1e-5);  // entry is 5
        assert(exitVal.value() >= 5.0 - 1e-5);  // exit >= entry

        std::cout << GREEN << "Test 55 PASSED: Logical OR constraint with comparison (!visit || (exit >= entry))" << RESET << std::endl;
    }

    // Test 56: Logical OR with <= comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint(y == 10.0);
        model.addConstraint(!flag || (x <= y));
        model.addConstraint(flag == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);

        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() <= yVal.value() + 1e-5);

        std::cout << GREEN << "Test 56 PASSED: Logical OR with <= comparison" << RESET << std::endl;
    }

    // Test 57: Logical OR with == comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);

        model.addConstraint(!flag || (x == 42.0));
        model.addConstraint(flag == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);

        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 42.0) < 1e-5);

        std::cout << GREEN << "Test 57 PASSED: Logical OR with == comparison" << RESET << std::endl;
    }

    // Test 58: Logical OR with > comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint(y == 10.0);
        model.addConstraint(!flag || (x > y));
        model.addConstraint(flag == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);

        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() > yVal.value() - 1e-5);

        std::cout << GREEN << "Test 58 PASSED: Logical OR with > comparison" << RESET << std::endl;
    }

    // Test 59: Logical OR with < comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint(y == 10.0);
        model.addConstraint(!flag || (x < y));
        model.addConstraint(flag == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);

        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() < yVal.value() + 1e-5);

        std::cout << GREEN << "Test 59 PASSED: Logical OR with < comparison" << RESET << std::endl;
    }

    // Test 60: Logical OR with != comparison
    {
        CP::Model model;
        const auto& flag = model.addBinaryVariable("flag");
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);

        model.addConstraint(!flag || (x != 50.0));
        model.addConstraint(flag == 1.0);
        model.addConstraint(x >= 49.0);
        model.addConstraint(x <= 51.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);

        assert(xVal.has_value());
        assert(std::abs(xVal.value() - 50.0) > 1e-6);

        std::cout << GREEN << "Test 60 PASSED: Logical OR with != comparison" << RESET << std::endl;
    }

    // Test 61: AND of two comparisons
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);
        const auto& y = model.addVariable(CP::Variable::Type::REAL, "y", 0.0, 100.0);

        model.addConstraint((x >= 10.0) && (y <= 20.0));

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);

        assert(xVal.has_value() && yVal.has_value());
        assert(xVal.value() >= 10.0 - 1e-5);
        assert(yVal.value() <= 20.0 + 1e-5);

        std::cout << GREEN << "Test 61 PASSED: AND of two comparisons" << RESET << std::endl;
    }

    // Test 62: OR of two comparisons
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);

        model.addConstraint((x <= 10.0) || (x >= 90.0));
        model.addConstraint(x >= 5.0);
        model.addConstraint(x <= 95.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);

        assert(xVal.has_value());
        // x should be either <= 10 or >= 90
        assert(xVal.value() <= 10.0 + 1e-5 || xVal.value() >= 90.0 - 1e-5);

        std::cout << GREEN << "Test 62 PASSED: OR of two comparisons" << RESET << std::endl;
    }

    // Test 63: Negation of comparison
    {
        CP::Model model;
        const auto& x = model.addVariable(CP::Variable::Type::REAL, "x", 0.0, 100.0);

        model.addConstraint(!(x >= 50.0));

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto xVal = solution.getVariableValue(x);

        assert(xVal.has_value());
        assert(xVal.value() < 50.0 + 1e-5);

        std::cout << GREEN << "Test 63 PASSED: Negation of comparison" << RESET << std::endl;
    }

    // Test 64: Simple deduced variable (b := a)
    {
        CP::Model model;
        const auto& a = model.addVariable(CP::Variable::Type::BOOLEAN, "a", 0.0, 1.0);
        const auto& b = model.addVariable(CP::Variable::Type::BOOLEAN, "b", a);

        model.addConstraint(a == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto aVal = solution.getVariableValue(a);
        auto bVal = solution.getVariableValue(b);

        assert(aVal.has_value());
        assert(bVal.has_value());
        assert(std::abs(aVal.value() - 1.0) < 1e-5);
        assert(std::abs(bVal.value() - 1.0) < 1e-5);

        std::cout << GREEN << "Test 64 PASSED: Simple deduced variable" << RESET << std::endl;
    }

    // Test 65: Deduced variable with arithmetic expression (c := a + b)
    {
        CP::Model model;
        const auto& a = model.addVariable(CP::Variable::Type::REAL, "a", 0.0, 10.0);
        const auto& b = model.addVariable(CP::Variable::Type::REAL, "b", 0.0, 10.0);
        const auto& c = model.addVariable(CP::Variable::Type::REAL, "c", a + b);

        model.addConstraint(a == 3.0);
        model.addConstraint(b == 5.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto aVal = solution.getVariableValue(a);
        auto bVal = solution.getVariableValue(b);
        auto cVal = solution.getVariableValue(c);

        assert(aVal.has_value());
        assert(bVal.has_value());
        assert(cVal.has_value());
        assert(std::abs(aVal.value() - 3.0) < 1e-5);
        assert(std::abs(bVal.value() - 5.0) < 1e-5);
        assert(std::abs(cVal.value() - 8.0) < 1e-5);

        std::cout << GREEN << "Test 65 PASSED: Deduced variable with arithmetic expression" << RESET << std::endl;
    }

    // Test 66: Deduced boolean variable in logical constraint
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

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto visitVal = solution.getVariableValue(visit);
        auto tokenflowVal = solution.getVariableValue(tokenflow);
        auto xVal = solution.getVariableValue(x);
        auto yVal = solution.getVariableValue(y);

        assert(visitVal.has_value());
        assert(tokenflowVal.has_value());
        assert(xVal.has_value());
        assert(yVal.has_value());
        assert(std::abs(visitVal.value() - 1.0) < 1e-5);
        assert(std::abs(tokenflowVal.value() - 1.0) < 1e-5);
        assert(std::abs(xVal.value() - 5.0) < 1e-5);
        assert(std::abs(yVal.value() - 3.0) < 1e-5);

        std::cout << GREEN << "Test 66 PASSED: Deduced boolean variable in logical constraint" << RESET << std::endl;
    }

    // Test 67: Deduced variable should fail when constraint is violated
    {
        CP::Model model;
        const auto& visit = model.addVariable(CP::Variable::Type::BOOLEAN, "visit", 0.0, 1.0);
        const auto& tokenflow = model.addVariable(CP::Variable::Type::BOOLEAN, "tokenflow", visit);
        const auto& exit = model.addVariable(CP::Variable::Type::REAL, "exit", 0.0, 10.0);
        const auto& value1 = model.addVariable(CP::Variable::Type::REAL, "value1", 0.0, 10.0);

        // Constraint: !tokenflow || (exit >= value1)
        model.addConstraint(!tokenflow || (exit >= value1));

        // Set visit = 1, which should make tokenflow = 1
        model.addConstraint(visit == 1.0);

        // Set exit = 0, value1 = 1, so exit >= value1 is violated
        // This should make the constraint fail
        model.addConstraint(exit == 0.0);
        model.addConstraint(value1 == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        // This problem should be infeasible
        assert(!result.has_value() || !result.value().errors().empty());

        std::cout << GREEN << "Test 67 PASSED: Deduced variable enforces constraint correctly (infeasible)" << RESET << std::endl;
    }

    // Test 68: Deduced variable from IndexedVariable access
    {
        CP::Model model;

        // Create indexed variables
        auto& array = model.addIndexedVariables(CP::Variable::Type::REAL, "array");
        array.emplace_back(5.0, 5.0);  // array[0] = 5
        array.emplace_back(10.0, 10.0); // array[1] = 10
        array.emplace_back(15.0, 15.0); // array[2] = 15

        // Create index variable
        const auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 0.0, 2.0);

        // Create deduced variable: value := array[index]
        const auto& value = model.addVariable(CP::Variable::Type::REAL, "value", array[index]);

        // Set index = 1, so value should be 10
        model.addConstraint(index == 1.0);

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto indexVal = solution.getVariableValue(index);
        auto valueVal = solution.getVariableValue(value);

        assert(indexVal.has_value());
        assert(valueVal.has_value());
        assert(std::abs(indexVal.value() - 1.0) < 1e-5);
        assert(std::abs(valueVal.value() - 10.0) < 1e-5);

        std::cout << GREEN << "Test 68 PASSED: Deduced variable from IndexedVariable access" << RESET << std::endl;
    }

    // Test 69: Deduced variable from IndexedVariable in constraint
    {
        CP::Model model;

        // Create indexed variables (like value_{Instance_1,Process_1},Instance)
        auto& processInstance = model.addIndexedVariables(CP::Variable::Type::REAL, "process_instance");
        processInstance.emplace_back(2.0, 2.0);  // processInstance[0] = 2

        // Create index variable (like data_index[Process_1]_{Instance_1,Activity_1,entry})
        const auto& dataIndex = model.addVariable(CP::Variable::Type::INTEGER, "data_index", 0.0, 0.0);

        // Create deduced variable (like value_{Instance_1,Activity_1,0},Instance)
        const auto& activityInstance = model.addVariable(CP::Variable::Type::REAL, "activity_instance", processInstance[dataIndex]);

        // This should constrain activity_instance = processInstance[0] = 2

        CP::SCIPSolver solver(model);
        auto result = solver.solve(model);

        assert(result.has_value());
        auto& solution = result.value();
        auto dataIndexVal = solution.getVariableValue(dataIndex);
        auto activityInstanceVal = solution.getVariableValue(activityInstance);

        assert(dataIndexVal.has_value());
        assert(activityInstanceVal.has_value());
        assert(std::abs(dataIndexVal.value() - 0.0) < 1e-5);
        assert(std::abs(activityInstanceVal.value() - 2.0) < 1e-5);

        std::cout << GREEN << "Test 69 PASSED: Deduced variable from IndexedVariable in constraint" << RESET << std::endl;
    }

    std::cout << "\n" << GREEN << "All 69 SCIP adapter tests PASSED" << RESET << std::endl;
    return 0;
}
