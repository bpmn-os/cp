#include "cp.h"
#include "scip/scip_adapter.h"
#include <scip/scip.h>
#include <iostream>
#include <cassert>

#define GREEN "\033[32m"
#define RESET "\033[0m"

int main() {
    // Test 1: Single integer variable
    {
        CP::Model model;
        const auto& x = model.addIntegerVariable("x");

        CP::SCIPSolver solver(model);
        SCIP* scip = solver.getScip();
        const auto& varMap = solver.getVarMap();

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
        const auto& varMap = solver.getVarMap();

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
        const auto& varMap = solver.getVarMap();

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

    std::cout << "\n" << GREEN << "All SCIP adapter tests PASSED" << RESET << std::endl;
    return 0;
}
