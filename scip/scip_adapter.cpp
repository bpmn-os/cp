#include "scip_adapter.h"
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <scip/expr_abs.h>
#include <scip/cons_linear.h>
#include <scip/cons_nonlinear.h>
#include <scip/expr_var.h>
#include <scip/expr_product.h>
#include <scip/expr_sum.h>
#include <limits>
#include <iostream>
#include <cstring>

namespace CP {

SCIPSolver::SCIPSolver(const Model& model, double epsilon)
  : epsilon_(epsilon)
{
  SCIPcreate(&scip_);
  SCIPincludeDefaultPlugins(scip_);
  SCIPcreateProbBasic(scip_, "cp_model");

  addSequences(model);
  addVariables(model);
  addIndexedVariables(model);
  addObjective(model);
  addConstraints(model);
}

SCIPSolver::~SCIPSolver() {
  if (scip_) {
    // Release all variables
    for (auto& [cpVar, scipVar] : variableMap_) {
      SCIPreleaseVar(scip_, &scipVar);
    }
    SCIPfree(&scip_);
  }
}

void SCIPSolver::addSequences(const Model& model) {
  for (const auto& sequence : model.getSequences()) {
    // Create sequence variables and collect SCIP vars
    std::vector<SCIP_VAR*> sequenceVariables;
    for (const Variable& variable : sequence.variables) {

      SCIP_VAR* scipVar;
      SCIPcreateVarBasic(scip_, &scipVar, variable.name.c_str(), 1, sequence.variables.size(), 0.0, SCIP_VARTYPE_INTEGER);
      SCIPaddVar(scip_, scipVar);
      variableMap_[&variable] = scipVar;
      sequenceVariables.push_back(scipVar);
    }

    // Add sequence constraint (alldifferent permutation of {1, ..., n})
    addSequenceConstraints(sequence.name, sequenceVariables);
  }
}

void SCIPSolver::addSequenceConstraints(const std::string& sequenceName, const std::vector<SCIP_VAR*>& sequenceVariables) {
  size_t n = sequenceVariables.size();
  int minValue = 1;
  int maxValue = n;

  // Binary matrix formulation for alldifferent
  // Create n×(maxValue-minValue+1) binary variables b[i][v] for each position i and value v
  std::vector<std::vector<SCIP_VAR*>> binaries(n);
  for (size_t i = 0; i < n; i++) {
    binaries[i].resize(maxValue - minValue + 1);
    for (int value = minValue; value <= maxValue; value++) {
      std::string binName = sequenceName + "_b[" + std::to_string(i) + "][" + std::to_string(value) + "]";
      SCIP_VAR* binVar;
      SCIPcreateVarBasic(scip_, &binVar, binName.c_str(), 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
      SCIPaddVar(scip_, binVar);
      binaries[i][value - minValue] = binVar;
    }
  }

  // Row constraints: sum_v b[i][v] = 1 for each position i
  for (size_t i = 0; i < n; i++) {
    SCIP_CONS* rowCons;
    std::vector<double> coeffs(binaries[i].size(), 1.0);
    std::string consName = sequenceName + "_row[" + std::to_string(i) + "]";
    SCIPcreateConsBasicLinear(scip_, &rowCons, consName.c_str(), binaries[i].size(), binaries[i].data(), coeffs.data(), 1.0, 1.0);
    SCIPaddCons(scip_, rowCons);
    SCIPreleaseCons(scip_, &rowCons);
  }

  // Column constraints: sum_i b[i][v] = 1 for each value v
  for (int value = minValue; value <= maxValue; value++) {
    SCIP_CONS* colCons;
    std::vector<SCIP_VAR*> colVars(n);
    std::vector<double> coeffs(n, 1.0);
    for (size_t i = 0; i < n; i++) {
      colVars[i] = binaries[i][value - minValue];
    }
    std::string consName = sequenceName + "_col[" + std::to_string(value) + "]";
    SCIPcreateConsBasicLinear(scip_, &colCons, consName.c_str(), n, colVars.data(), coeffs.data(), 1.0, 1.0);
    SCIPaddCons(scip_, colCons);
    SCIPreleaseCons(scip_, &colCons);
  }

  // Link constraints: x[i] = sum_v (v * b[i][v])
  for (size_t i = 0; i < n; i++) {
    SCIP_VAR* scipVar = sequenceVariables[i];

    SCIP_CONS* linkCons;
    std::vector<SCIP_VAR*> linkVars;
    std::vector<double> linkCoeffs;

    // Add x[i] with coefficient -1
    linkVars.push_back(scipVar);
    linkCoeffs.push_back(-1.0);

    // Add b[i][v] with coefficient v
    for (int value = minValue; value <= maxValue; value++) {
      linkVars.push_back(binaries[i][value - minValue]);
      linkCoeffs.push_back(static_cast<double>(value));
    }

    std::string consName = sequenceName + "_link[" + std::to_string(i) + "]";
    SCIPcreateConsBasicLinear(scip_, &linkCons, consName.c_str(), linkVars.size(), linkVars.data(), linkCoeffs.data(), -epsilon_, epsilon_);
    SCIPaddCons(scip_, linkCons);
    SCIPreleaseCons(scip_, &linkCons);
  }

  // Release binary variables
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < binaries[i].size(); j++) {
      SCIPreleaseVar(scip_, &binaries[i][j]);
    }
  }
}

void SCIPSolver::addVariables(const Model& model) {
  for (const auto& var : model.getVariables()) {
    SCIP_VARTYPE vartype;

    switch (var.type) {
      case Variable::Type::BOOLEAN:
        vartype = SCIP_VARTYPE_BINARY;
        break;
      case Variable::Type::INTEGER:
        vartype = SCIP_VARTYPE_INTEGER;
        break;
      case Variable::Type::REAL:
        vartype = SCIP_VARTYPE_CONTINUOUS;
        break;
    }

    // Convert C++ limits to SCIP infinity
    double lowerBound = (var.lowerBound == std::numeric_limits<double>::lowest())
                        ? -SCIPinfinity(scip_) : var.lowerBound;
    double upperBound = (var.upperBound == std::numeric_limits<double>::max())
                        ? SCIPinfinity(scip_) : var.upperBound;

    SCIP_VAR* scipVar;
    SCIPcreateVarBasic(scip_, &scipVar, var.name.c_str(), lowerBound, upperBound, 0.0, vartype);
    SCIPaddVar(scip_, scipVar);
    variableMap_[&var] = scipVar;
  }
}

void SCIPSolver::addIndexedVariables(const Model& model) {
  for (const auto& indexedVars : model.getIndexedVariables()) {
    for (const auto& var : indexedVars) {
      SCIP_VARTYPE vartype;

      switch (var.type) {
        case Variable::Type::BOOLEAN:
          vartype = SCIP_VARTYPE_BINARY;
          break;
        case Variable::Type::INTEGER:
          vartype = SCIP_VARTYPE_INTEGER;
          break;
        case Variable::Type::REAL:
          vartype = SCIP_VARTYPE_CONTINUOUS;
          break;
      }

      // Convert C++ limits to SCIP infinity
      double lowerBound = (var.lowerBound == std::numeric_limits<double>::lowest())
                          ? -SCIPinfinity(scip_) : var.lowerBound;
      double upperBound = (var.upperBound == std::numeric_limits<double>::max())
                          ? SCIPinfinity(scip_) : var.upperBound;

      SCIP_VAR* scipVar;
      SCIPcreateVarBasic(scip_, &scipVar, var.name.c_str(), lowerBound, upperBound, 0.0, vartype);
      SCIPaddVar(scip_, scipVar);
      variableMap_[&var] = scipVar;
    }
  }
}

SCIP_EXPR* SCIPSolver::createBoolExpr(SCIP_EXPR* expr) {
  // Convert expression to boolean: 0 if abs(expr) < epsilon, 1 if abs(expr) >= epsilon
  // Using epsilon-based algebraic formulation (no big-M)

  // Create binary variable b
  SCIP_VAR* boolVar;
  SCIPcreateVarBasic(scip_, &boolVar, "bool_aux", 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
  SCIPaddVar(scip_, boolVar);

  // Create abs(expr)
  SCIP_EXPR* absExpr;
  SCIPcreateExprAbs(scip_, &absExpr, expr, nullptr, nullptr);

  // Constraint 1: abs(expr) >= 1.1 * epsilon * b
  // If b=1, then abs(expr) >= 1.1 * epsilon (strict side with tolerance)
  SCIP_EXPR* boolVarExpr1;
  SCIPcreateExprVar(scip_, &boolVarExpr1, boolVar, nullptr, nullptr);

  SCIP_EXPR* epsilonB;
  double coeff1 = 1.1 * epsilon_;
  SCIPcreateExprSum(scip_, &epsilonB, 1, &boolVarExpr1, &coeff1, 0.0, nullptr, nullptr);

  SCIP_EXPR* diff1;
  SCIP_EXPR* children1[] = { absExpr, epsilonB };
  double coeffs1[] = { 1.0, -1.0 };
  SCIPcreateExprSum(scip_, &diff1, 2, children1, coeffs1, 0.0, nullptr, nullptr);

  SCIP_CONS* cons1;
  SCIPcreateConsBasicNonlinear(scip_, &cons1, "bool_lower", diff1, 0.0, SCIPinfinity(scip_));
  SCIPaddCons(scip_, cons1);
  SCIPreleaseCons(scip_, &cons1);
  SCIPreleaseExpr(scip_, &diff1);
  SCIPreleaseExpr(scip_, &epsilonB);
  SCIPreleaseExpr(scip_, &boolVarExpr1);

  // Constraint 2: (1-b) * (abs(expr) - epsilon) <= 0
  // If b=0, then abs(expr) <= epsilon (forces small values when b=0)
  // If b=1, constraint is 0 <= 0 (always satisfied)
  SCIP_EXPR* absExpr2;
  SCIPduplicateExpr(scip_, absExpr, &absExpr2, nullptr, nullptr, nullptr, nullptr);

  SCIP_EXPR* boolVarExpr2;
  SCIPcreateExprVar(scip_, &boolVarExpr2, boolVar, nullptr, nullptr);

  // (1 - b)
  SCIP_EXPR* oneMinusB;
  double coeff2 = -1.0;
  SCIPcreateExprSum(scip_, &oneMinusB, 1, &boolVarExpr2, &coeff2, 1.0, nullptr, nullptr);

  // (abs(expr) - epsilon)
  SCIP_EXPR* absMinusEps;
  SCIPcreateExprSum(scip_, &absMinusEps, 1, &absExpr2, nullptr, -epsilon_, nullptr, nullptr);

  // (1-b) * (abs(expr) - epsilon)
  SCIP_EXPR* product;
  SCIP_EXPR* prodChildren[] = { oneMinusB, absMinusEps };
  SCIPcreateExprProduct(scip_, &product, 2, prodChildren, 1.0, nullptr, nullptr);

  // (1-b) * (abs(expr) - epsilon) <= 0
  SCIP_CONS* cons2;
  SCIPcreateConsBasicNonlinear(scip_, &cons2, "bool_upper", product, -SCIPinfinity(scip_), 0.0);
  SCIPaddCons(scip_, cons2);
  SCIPreleaseCons(scip_, &cons2);
  SCIPreleaseExpr(scip_, &product);
  SCIPreleaseExpr(scip_, &absMinusEps);
  SCIPreleaseExpr(scip_, &oneMinusB);
  SCIPreleaseExpr(scip_, &boolVarExpr2);
  SCIPreleaseExpr(scip_, &absExpr2);
  SCIPreleaseExpr(scip_, &absExpr);

  // Return expression for boolean variable
  SCIP_EXPR* resultExpr;
  SCIPcreateExprVar(scip_, &resultExpr, boolVar, nullptr, nullptr);
  SCIPreleaseVar(scip_, &boolVar);

  return resultExpr;
}

void SCIPSolver::addObjective(const Model& model) {
  // Set objective sense
  if (model.getObjectiveSense() == Model::ObjectiveSense::MINIMIZE) {
    SCIPsetObjsense(scip_, SCIP_OBJSENSE_MINIMIZE);
  }
  else if (model.getObjectiveSense() == Model::ObjectiveSense::MAXIMIZE) {
    SCIPsetObjsense(scip_, SCIP_OBJSENSE_MAXIMIZE);
  }
  else {
    // FEASIBLE - no objective
    return;
  }

  const auto& objective = model.getObjective();

  // Build objective expression - wrap in operand
  Operand objOperand = objective;
  auto objExpr = buildExpr(objOperand);
  if (!objExpr) {
    return;
  }

  // Create auxiliary variable for objective with coefficient 1.0
  SCIP_VAR* objVar;
  SCIPcreateVarBasic(scip_, &objVar, "obj", -SCIPinfinity(scip_), SCIPinfinity(scip_), 1.0, SCIP_VARTYPE_CONTINUOUS);
  SCIPaddVar(scip_, objVar);

  // Create constraint: objVar == objective_expression
  SCIP_EXPR* objVarExpr;
  SCIPcreateExprVar(scip_, &objVarExpr, objVar, nullptr, nullptr);

  SCIP_EXPR* diffExpr;
  SCIP_EXPR* children[] = { objVarExpr, objExpr.value() };
  double coeffs[] = { 1.0, -1.0 };
  SCIPcreateExprSum(scip_, &diffExpr, 2, children, coeffs, 0.0, nullptr, nullptr);

  SCIP_CONS* objCons;
  SCIPcreateConsBasicNonlinear(scip_, &objCons, "obj_constraint", diffExpr, 0.0, 0.0);
  SCIPaddCons(scip_, objCons);

  SCIPreleaseCons(scip_, &objCons);
  SCIPreleaseExpr(scip_, &diffExpr);
  SCIPreleaseExpr(scip_, &objVarExpr);
  SCIPreleaseExpr(scip_, &objExpr.value());
  SCIPreleaseVar(scip_, &objVar);
}

std::expected<SCIP_EXPR*, std::string> SCIPSolver::buildExpr(const Operand& operand) {
  // Handle constants
  if (std::holds_alternative<double>(operand)) {
    SCIP_EXPR* expr;
    SCIPcreateExprValue(scip_, &expr, std::get<double>(operand), nullptr, nullptr);
    return expr;
  }

  // Handle variables
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(operand)) {
    const Variable& var = std::get<std::reference_wrapper<const Variable>>(operand).get();
    auto it = variableMap_.find(&var);
    if (it == variableMap_.end()) {
      return std::unexpected("Variable not found in varMap");
    }
    SCIP_EXPR* expr;
    SCIPcreateExprVar(scip_, &expr, it->second, nullptr, nullptr);
    return expr;
  }

  // Handle indexed variables (element constraint: result = array[index])
  if (std::holds_alternative<IndexedVariable>(operand)) {
    const IndexedVariable& indexedVar = std::get<IndexedVariable>(operand);
    const IndexedVariables& container = indexedVar.container.get();
    const Variable& indexVar = indexedVar.index.get();

    // Get index SCIP variable
    auto indexIt = variableMap_.find(&indexVar);
    if (indexIt == variableMap_.end()) {
      return std::unexpected("Index variable not found in varMap");
    }
    SCIP_VAR* scipIndexVar = indexIt->second;

    // Get array SCIP variables
    std::vector<SCIP_VAR*> arrayVars;
    for (const auto& var : container) {
      auto varIt = variableMap_.find(&var);
      if (varIt == variableMap_.end()) {
        return std::unexpected("Array variable not found in varMap");
      }
      arrayVars.push_back(varIt->second);
    }

    // Create result variable (unbounded - let constraint determine valid range)
    SCIP_VAR* resultVar;
    std::string resultName = container.name + "[" + indexVar.name + "]_result";
    SCIPcreateVarBasic(scip_, &resultVar, resultName.c_str(), -SCIPinfinity(scip_), SCIPinfinity(scip_), 0.0, SCIP_VARTYPE_CONTINUOUS);
    SCIPaddVar(scip_, resultVar);

    // Add element constraint (0-based indexing)
    std::string elemName = container.name + "[" + indexVar.name + "]";
    SCIP_EXPR* resultExpr = addIndexingConstraints(elemName, arrayVars, scipIndexVar, resultVar);

    SCIPreleaseVar(scip_, &resultVar);
    return resultExpr;
  }

  // Handle expressions
  if (std::holds_alternative<Expression>(operand)) {
    const Expression& cpExpr = std::get<Expression>(operand);
    using enum Expression::Operator;

    // Base case: none operator
    if (cpExpr._operator == none && cpExpr.operands.size() == 1) {
      return buildExpr(cpExpr.operands[0]);
    }

    // Negate: -expr
    if (cpExpr._operator == negate && cpExpr.operands.size() == 1) {
      auto subExpr = buildExpr(cpExpr.operands[0]);
      if (!subExpr) return subExpr;

      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { subExpr.value() };
      double coeffs[] = { -1.0 };
      SCIPcreateExprSum(scip_, &expr, 1, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &children[0]);
      return expr;
    }

    // Addition: a + b
    if (cpExpr._operator == add && cpExpr.operands.size() == 2) {
      auto left = buildExpr(cpExpr.operands[0]);
      auto right = buildExpr(cpExpr.operands[1]);
      if (!left) return left;
      if (!right) return right;

      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { left.value(), right.value() };
      double coeffs[] = { 1.0, 1.0 };
      SCIPcreateExprSum(scip_, &expr, 2, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &children[0]);
      SCIPreleaseExpr(scip_, &children[1]);
      return expr;
    }

    // Subtraction: a - b
    if (cpExpr._operator == subtract && cpExpr.operands.size() == 2) {
      auto left = buildExpr(cpExpr.operands[0]);
      auto right = buildExpr(cpExpr.operands[1]);
      if (!left) return left;
      if (!right) return right;

      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { left.value(), right.value() };
      double coeffs[] = { 1.0, -1.0 };
      SCIPcreateExprSum(scip_, &expr, 2, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &children[0]);
      SCIPreleaseExpr(scip_, &children[1]);
      return expr;
    }

    // Multiplication: a * b
    if (cpExpr._operator == multiply && cpExpr.operands.size() == 2) {
      auto left = buildExpr(cpExpr.operands[0]);
      auto right = buildExpr(cpExpr.operands[1]);
      if (!left) return left;
      if (!right) return right;

      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { left.value(), right.value() };
      SCIPcreateExprProduct(scip_, &expr, 2, children, 1.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &children[0]);
      SCIPreleaseExpr(scip_, &children[1]);
      return expr;
    }

    // Division: a / b
    if (cpExpr._operator == divide && cpExpr.operands.size() == 2) {
      auto numerator = buildExpr(cpExpr.operands[0]);
      auto denominator = buildExpr(cpExpr.operands[1]);
      if (!numerator) return numerator;
      if (!denominator) return denominator;

      // Division is a * b^(-1)
      SCIP_EXPR* powExpr;
      SCIPcreateExprPow(scip_, &powExpr, denominator.value(), -1.0, nullptr, nullptr);

      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { numerator.value(), powExpr };
      SCIPcreateExprProduct(scip_, &expr, 2, children, 1.0, nullptr, nullptr);

      SCIPreleaseExpr(scip_, &children[0]);
      SCIPreleaseExpr(scip_, &children[1]);
      SCIPreleaseExpr(scip_, &denominator.value());
      return expr;
    }

    // Logical NOT: !a  => 1 - bool(a)
    if (cpExpr._operator == logical_not && cpExpr.operands.size() == 1) {
      auto subExpr = buildExpr(cpExpr.operands[0]);
      if (!subExpr) return subExpr;

      // Convert to boolean first
      SCIP_EXPR* boolExpr = createBoolExpr(subExpr.value());
      SCIPreleaseExpr(scip_, &subExpr.value());

      // NOT: 1 - bool(a)
      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { boolExpr };
      double coeffs[] = { -1.0 };
      SCIPcreateExprSum(scip_, &expr, 1, children, coeffs, 1.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &boolExpr);
      return expr;
    }

    // Logical AND: a && b  => bool(a) * bool(b)
    if (cpExpr._operator == logical_and && cpExpr.operands.size() == 2) {
      auto left = buildExpr(cpExpr.operands[0]);
      auto right = buildExpr(cpExpr.operands[1]);
      if (!left) return left;
      if (!right) return right;

      // Convert to booleans first
      SCIP_EXPR* boolLeft = createBoolExpr(left.value());
      SCIP_EXPR* boolRight = createBoolExpr(right.value());
      SCIPreleaseExpr(scip_, &left.value());
      SCIPreleaseExpr(scip_, &right.value());

      // AND: bool(a) * bool(b)
      SCIP_EXPR* expr;
      SCIP_EXPR* children[] = { boolLeft, boolRight };
      SCIPcreateExprProduct(scip_, &expr, 2, children, 1.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &boolLeft);
      SCIPreleaseExpr(scip_, &boolRight);
      return expr;
    }

    // Logical OR: a || b  => bool(bool(a) + bool(b))
    if (cpExpr._operator == logical_or && cpExpr.operands.size() == 2) {
      auto left = buildExpr(cpExpr.operands[0]);
      auto right = buildExpr(cpExpr.operands[1]);
      if (!left) return left;
      if (!right) return right;

      // Convert to booleans first
      SCIP_EXPR* boolLeft = createBoolExpr(left.value());
      SCIP_EXPR* boolRight = createBoolExpr(right.value());
      SCIPreleaseExpr(scip_, &left.value());
      SCIPreleaseExpr(scip_, &right.value());

      // OR: bool(a) + bool(b)
      SCIP_EXPR* sumExpr;
      SCIP_EXPR* sumChildren[] = { boolLeft, boolRight };
      double coeffs[] = { 1.0, 1.0 };
      SCIPcreateExprSum(scip_, &sumExpr, 2, sumChildren, coeffs, 0.0, nullptr, nullptr);

      SCIPreleaseExpr(scip_, &boolLeft);
      SCIPreleaseExpr(scip_, &boolRight);

      // Convert sum back to boolean
      SCIP_EXPR* expr = createBoolExpr(sumExpr);
      SCIPreleaseExpr(scip_, &sumExpr);
      return expr;
    }

    // Custom operators: max, min, sum, pow, etc.
    if (cpExpr._operator == custom && cpExpr.operands.size() >= 2) {
      // First operand is the operator index
      if (!std::holds_alternative<size_t>(cpExpr.operands[0])) {
        return std::unexpected("Custom operator index missing");
      }
      size_t opIndex = std::get<size_t>(cpExpr.operands[0]);
      std::string opName = Expression::customOperators[opIndex];

      SCIP_EXPR* expr = nullptr;

      // Handle specific custom operators
      if (opName == "pow" && cpExpr.operands.size() == 3) {
        // pow(a, b) - if b is constant
        if (std::holds_alternative<double>(cpExpr.operands[2])) {
          double exponent = std::get<double>(cpExpr.operands[2]);
          auto baseExpr = buildExpr(cpExpr.operands[1]);
          if (!baseExpr) return baseExpr;

          SCIPcreateExprPow(scip_, &expr, baseExpr.value(), exponent, nullptr, nullptr);
          SCIPreleaseExpr(scip_, &baseExpr.value());
          return expr;
        }
        else {
          return std::unexpected("pow with non-constant exponent not supported");
        }
      }

      // For other operators, build all child expressions
      std::vector<SCIP_EXPR*> children;
      for (size_t i = 1; i < cpExpr.operands.size(); i++) {
        auto childExpr = buildExpr(cpExpr.operands[i]);
        if (!childExpr) {
          // Cleanup already created expressions
          for (auto child : children) {
            SCIPreleaseExpr(scip_, &child);
          }
          return childExpr;
        }
        children.push_back(childExpr.value());
      }

      if (opName == "min" || opName == "max") {
        // min/max(a, b, ...) - use auxiliary variable with constraints
        // Create auxiliary variable for result
        SCIP_VAR* auxVar;
        SCIPcreateVarBasic(scip_, &auxVar, (opName + "_aux").c_str(), -SCIPinfinity(scip_), SCIPinfinity(scip_), 0.0, SCIP_VARTYPE_CONTINUOUS);
        SCIPaddVar(scip_, auxVar);

        // For each child, add constraint: auxVar <= child (for min) or auxVar >= child (for max)
        for (auto child : children) {
          SCIP_EXPR* auxExpr;
          SCIPcreateExprVar(scip_, &auxExpr, auxVar, nullptr, nullptr);

          // Create constraint: auxVar - child <= 0 (for min) or child - auxVar <= 0 (for max)
          SCIP_EXPR* diffExpr;
          if (opName == "min") {
            // auxVar <= child  =>  auxVar - child <= 0
            SCIP_EXPR* exprs[] = { auxExpr, child };
            double coeffs[] = { 1.0, -1.0 };
            SCIPcreateExprSum(scip_, &diffExpr, 2, exprs, coeffs, 0.0, nullptr, nullptr);
          }
          else { // max
            // auxVar >= child  =>  child - auxVar <= 0
            SCIP_EXPR* exprs[] = { child, auxExpr };
            double coeffs[] = { 1.0, -1.0 };
            SCIPcreateExprSum(scip_, &diffExpr, 2, exprs, coeffs, 0.0, nullptr, nullptr);
          }

          SCIP_CONS* cons;
          SCIPcreateConsBasicNonlinear(scip_, &cons, (opName + "_bound").c_str(),
                        diffExpr, -SCIPinfinity(scip_), 0.0);
          SCIPaddCons(scip_, cons);
          SCIPreleaseCons(scip_, &cons);
          SCIPreleaseExpr(scip_, &diffExpr);
          SCIPreleaseExpr(scip_, &auxExpr);
        }

        // Add disjunctive constraint: auxVar = child_1 OR auxVar = child_2 OR ...
        // This is modeled as: (auxVar - child_1) * (auxVar - child_2) * ... = 0
        std::vector<SCIP_EXPR*> products;
        for (auto child : children) {
          SCIP_EXPR* auxExprCopy;
          SCIPcreateExprVar(scip_, &auxExprCopy, auxVar, nullptr, nullptr);

          SCIP_EXPR* diffExpr;
          SCIP_EXPR* exprs[] = { auxExprCopy, child };
          double coeffs[] = { 1.0, -1.0 };
          SCIPcreateExprSum(scip_, &diffExpr, 2, exprs, coeffs, 0.0, nullptr, nullptr);

          products.push_back(diffExpr);
          SCIPreleaseExpr(scip_, &auxExprCopy);
        }

        // Create product: (auxVar - child_1) * (auxVar - child_2) * ...
        SCIP_EXPR* productExpr;
        SCIPcreateExprProduct(scip_, &productExpr, products.size(), products.data(),
                   1.0, nullptr, nullptr);

        SCIP_CONS* eqCons;
        SCIPcreateConsBasicNonlinear(scip_, &eqCons, (opName + "_eq").c_str(),
                      productExpr, 0.0, 0.0);
        SCIPaddCons(scip_, eqCons);
        SCIPreleaseCons(scip_, &eqCons);
        SCIPreleaseExpr(scip_, &productExpr);

        for (auto prod : products) {
          SCIPreleaseExpr(scip_, &prod);
        }

        // Create expression for auxiliary variable
        SCIPcreateExprVar(scip_, &expr, auxVar, nullptr, nullptr);
        SCIPreleaseVar(scip_, &auxVar);

        // Release children
        for (auto child : children) {
          SCIPreleaseExpr(scip_, &child);
        }

        return expr;
      }
      else if (opName == "sum") {
        // sum(a, b, ...) = a + b + ...
        std::vector<double> coeffs(children.size(), 1.0);
        SCIPcreateExprSum(scip_, &expr, children.size(), children.data(), coeffs.data(), 0.0, nullptr, nullptr);
      }
      else if (opName == "avg") {
        // avg = sum / count
        std::vector<double> coeffs(children.size(), 1.0 / children.size());
        SCIPcreateExprSum(scip_, &expr, children.size(), children.data(), coeffs.data(), 0.0, nullptr, nullptr);
      }
      else if (opName == "if_then_else") {
        // if_then_else(c, v1, v2) = c * v1 + (1 - c) * v2
        if (children.size() != 3) {
          for (auto child : children) {
            SCIPreleaseExpr(scip_, &child);
          }
          return std::unexpected("if_then_else requires exactly 3 operands");
        }

        SCIP_EXPR* condition = children[0];
        SCIP_EXPR* ifValue = children[1];
        SCIP_EXPR* elseValue = children[2];

        // Create c * v1
        SCIP_EXPR* term1;
        SCIP_EXPR* factors1[] = { condition, ifValue };
        SCIPcreateExprProduct(scip_, &term1, 2, factors1, 1.0, nullptr, nullptr);

        // Create (1 - c)
        SCIP_EXPR* conditionCopy;
        SCIPduplicateExpr(scip_, condition, &conditionCopy, nullptr, nullptr, nullptr, nullptr);
        SCIP_EXPR* oneMinusC;
        double coeff = -1.0;
        SCIPcreateExprSum(scip_, &oneMinusC, 1, &conditionCopy, &coeff, 1.0, nullptr, nullptr);
        SCIPreleaseExpr(scip_, &conditionCopy);

        // Create (1 - c) * v2
        SCIP_EXPR* term2;
        SCIP_EXPR* factors2[] = { oneMinusC, elseValue };
        SCIPcreateExprProduct(scip_, &term2, 2, factors2, 1.0, nullptr, nullptr);
        SCIPreleaseExpr(scip_, &oneMinusC);

        // Create term1 + term2
        SCIP_EXPR* terms[] = { term1, term2 };
        double coeffs[] = { 1.0, 1.0 };
        SCIPcreateExprSum(scip_, &expr, 2, terms, coeffs, 0.0, nullptr, nullptr);

        SCIPreleaseExpr(scip_, &term1);
        SCIPreleaseExpr(scip_, &term2);
      }
      else if (opName == "at") {
        // at(index, val1, val2, val3, ...) returns val[index]
        // First child is index, remaining children are array values
        if (children.size() < 2) {
          for (auto child : children) {
            SCIPreleaseExpr(scip_, &child);
          }
          return std::unexpected("at requires at least 2 operands (index and at least one value)");
        }

        SCIP_EXPR* indexExpr = children[0];
        std::vector<SCIP_EXPR*> arrayExprs(children.begin() + 1, children.end());

        // Create index variable from expression
        SCIP_VAR* indexVar;
        std::string indexName = "at_index_" + std::to_string(reinterpret_cast<uintptr_t>(this));
        SCIPcreateVarBasic(scip_, &indexVar, indexName.c_str(),
                  0, arrayExprs.size() - 1, 0.0, SCIP_VARTYPE_INTEGER);
        SCIPaddVar(scip_, indexVar);

        // Constrain index variable to equal the index expression
        SCIP_EXPR* indexVarExpr;
        SCIPcreateExprVar(scip_, &indexVarExpr, indexVar, nullptr, nullptr);
        SCIP_EXPR* indexDiff;
        SCIP_EXPR* indexExprs[] = { indexVarExpr, indexExpr };
        double indexCoeffs[] = { 1.0, -1.0 };
        SCIPcreateExprSum(scip_, &indexDiff, 2, indexExprs, indexCoeffs, 0.0, nullptr, nullptr);
        SCIP_CONS* indexCons;
        SCIPcreateConsBasicNonlinear(scip_, &indexCons, "at_index_eq", indexDiff, 0.0, 0.0);
        SCIPaddCons(scip_, indexCons);
        SCIPreleaseCons(scip_, &indexCons);
        SCIPreleaseExpr(scip_, &indexDiff);
        SCIPreleaseExpr(scip_, &indexVarExpr);

        // Create array variables from expressions (need to materialize expressions as variables)
        std::vector<SCIP_VAR*> arrayVars;
        for (size_t i = 0; i < arrayExprs.size(); i++) {
          SCIP_VAR* arrayVar;
          std::string arrayVarName = "at_array_" + std::to_string(i) + "_" + std::to_string(reinterpret_cast<uintptr_t>(this));
          SCIPcreateVarBasic(scip_, &arrayVar, arrayVarName.c_str(), -SCIPinfinity(scip_), SCIPinfinity(scip_), 0.0, SCIP_VARTYPE_CONTINUOUS);
          SCIPaddVar(scip_, arrayVar);

          // Constrain array variable to equal the expression
          SCIP_EXPR* arrayVarExpr;
          SCIPcreateExprVar(scip_, &arrayVarExpr, arrayVar, nullptr, nullptr);
          SCIP_EXPR* arrayDiff;
          SCIP_EXPR* arrayDiffExprs[] = { arrayVarExpr, arrayExprs[i] };
          double arrayCoeffs[] = { 1.0, -1.0 };
          SCIPcreateExprSum(scip_, &arrayDiff, 2, arrayDiffExprs, arrayCoeffs, 0.0, nullptr, nullptr);
          SCIP_CONS* arrayCons;
          std::string arrayConsName = "at_array_eq_" + std::to_string(i);
          SCIPcreateConsBasicNonlinear(scip_, &arrayCons, arrayConsName.c_str(), arrayDiff, 0.0, 0.0);
          SCIPaddCons(scip_, arrayCons);
          SCIPreleaseCons(scip_, &arrayCons);
          SCIPreleaseExpr(scip_, &arrayDiff);
          SCIPreleaseExpr(scip_, &arrayVarExpr);

          arrayVars.push_back(arrayVar);
        }

        // Create result variable
        SCIP_VAR* resultVar;
        std::string resultName = "at_result_" + std::to_string(reinterpret_cast<uintptr_t>(this));
        SCIPcreateVarBasic(scip_, &resultVar, resultName.c_str(),
                  -SCIPinfinity(scip_), SCIPinfinity(scip_), 0.0, SCIP_VARTYPE_CONTINUOUS);
        SCIPaddVar(scip_, resultVar);

        // Add element constraint
        std::string elemName = "at_elem_" + std::to_string(reinterpret_cast<uintptr_t>(this));
        SCIP_EXPR* resultExpr = addIndexingConstraints(elemName, arrayVars, indexVar, resultVar);

        // Release variables
        SCIPreleaseVar(scip_, &indexVar);
        for (auto var : arrayVars) {
          SCIPreleaseVar(scip_, &var);
        }
        SCIPreleaseVar(scip_, &resultVar);

        // Release children
        for (auto child : children) {
          SCIPreleaseExpr(scip_, &child);
        }

        return resultExpr;
      }
      else if (opName == "n_ary_if") {
        // n_ary_if(c1, v1, c2, v2, ..., cN, vN, vElse)
        // result = c1*v1 + (1-c1)*c2*v2 + (1-c1)*(1-c2)*c3*v3 + ... + (1-c1)*...*(1-cN)*vElse

        if (children.size() % 2 != 1) {
          for (auto child : children) {
            SCIPreleaseExpr(scip_, &child);
          }
          return std::unexpected("n_ary_if requires an odd number of operands");
        }

        // Build terms: each term is (product of (1-cj) for j<i) * ci * vi
        std::vector<SCIP_EXPR*> terms;

        size_t numConditions = children.size() / 2;
        for (size_t i = 0; i < numConditions; i++) {
          SCIP_EXPR* condition = children[2*i];
          SCIP_EXPR* value = children[2*i + 1];

          // Build product of (1 - cj) for all j < i
          SCIP_EXPR* prefix = nullptr;
          if (i == 0) {
            // First term: just c1 * v1
            SCIP_EXPR* factors[] = { condition, value };
            SCIPcreateExprProduct(scip_, &prefix, 2, factors, 1.0, nullptr, nullptr);
          }
          else {
            // Build (1-c1) * (1-c2) * ... * (1-c_{i-1}) * ci * vi
            std::vector<SCIP_EXPR*> factors;

            // Add (1 - cj) for j < i
            for (size_t j = 0; j < i; j++) {
              SCIP_EXPR* cj = children[2*j];
              SCIP_EXPR* cjCopy;
              SCIPduplicateExpr(scip_, cj, &cjCopy, nullptr, nullptr, nullptr, nullptr);

              // Create (1 - cj)
              SCIP_EXPR* oneMinus;
              double coeff = -1.0;
              SCIPcreateExprSum(scip_, &oneMinus, 1, &cjCopy, &coeff, 1.0, nullptr, nullptr);
              factors.push_back(oneMinus);
              SCIPreleaseExpr(scip_, &cjCopy);
            }

            // Add ci
            factors.push_back(condition);

            // Add vi
            factors.push_back(value);

            // Create product
            SCIPcreateExprProduct(scip_, &prefix, factors.size(), factors.data(), 1.0, nullptr, nullptr);

            // Release (1-cj) expressions
            for (size_t j = 0; j < i; j++) {
              SCIPreleaseExpr(scip_, &factors[j]);
            }
          }

          terms.push_back(prefix);
        }

        // Add else term: (1-c1) * (1-c2) * ... * (1-cN) * vElse
        SCIP_EXPR* elseValue = children.back();
        std::vector<SCIP_EXPR*> elseFactors;

        for (size_t j = 0; j < numConditions; j++) {
          SCIP_EXPR* cj = children[2*j];
          SCIP_EXPR* cjCopy;
          SCIPduplicateExpr(scip_, cj, &cjCopy, nullptr, nullptr, nullptr, nullptr);

          // Create (1 - cj)
          SCIP_EXPR* oneMinus;
          double coeff = -1.0;
          SCIPcreateExprSum(scip_, &oneMinus, 1, &cjCopy, &coeff, 1.0, nullptr, nullptr);
          elseFactors.push_back(oneMinus);
          SCIPreleaseExpr(scip_, &cjCopy);
        }

        elseFactors.push_back(elseValue);

        SCIP_EXPR* elseTerm;
        SCIPcreateExprProduct(scip_, &elseTerm, elseFactors.size(), elseFactors.data(), 1.0, nullptr, nullptr);
        terms.push_back(elseTerm);

        // Release (1-cj) expressions
        for (size_t j = 0; j < numConditions; j++) {
          SCIPreleaseExpr(scip_, &elseFactors[j]);
        }

        // Sum all terms
        std::vector<double> coeffs(terms.size(), 1.0);
        SCIPcreateExprSum(scip_, &expr, terms.size(), terms.data(), coeffs.data(), 0.0, nullptr, nullptr);

        // Release terms
        for (auto term : terms) {
          SCIPreleaseExpr(scip_, &term);
        }
      }
      else {
        // Unsupported custom operator
        for (auto child : children) {
          SCIPreleaseExpr(scip_, &child);
        }
        return std::unexpected("Unsupported custom operator: " + opName);
      }

      // Release children (SCIP keeps its own references)
      for (auto child : children) {
        SCIPreleaseExpr(scip_, &child);
      }

      return expr;
    }

    return std::unexpected("Unsupported expression operator");
  }

  return std::unexpected("Unsupported operand type");
}

void SCIPSolver::addConstraints(const Model& model) {
  const auto& constraints = model.getConstraints();
  for (size_t i = 0; i < constraints.size(); i++) {
    const auto& constraint = constraints[i];
    using enum Expression::Operator;

    // Handle comparison operators: <=, >=, ==, <, >, !=
    if (constraint._operator == less_or_equal ||
      constraint._operator == greater_or_equal ||
      constraint._operator == equal ||
      constraint._operator == less_than ||
      constraint._operator == greater_than ||
      constraint._operator == not_equal) {

      if (constraint.operands.size() != 2) {
        continue; // Skip malformed constraints
      }

      // Build expression for lhs - rhs
      auto leftExpr = buildExpr(constraint.operands[0]);
      auto rightExpr = buildExpr(constraint.operands[1]);

      if (!leftExpr || !rightExpr) {
        continue; // Skip if expression building failed
      }

      // Create lhs - rhs expression
      SCIP_EXPR* diffExpr;
      SCIP_EXPR* children[] = { leftExpr.value(), rightExpr.value() };
      double coeffs[] = { 1.0, -1.0 };
      SCIPcreateExprSum(scip_, &diffExpr, 2, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip_, &children[0]);
      SCIPreleaseExpr(scip_, &children[1]);

      // Determine bounds: lhs - rhs {<=, >=, ==} 0
      double lhs = -SCIPinfinity(scip_);
      double rhs = SCIPinfinity(scip_);

      std::string consName = "cons_" + std::to_string(i);

      if (constraint._operator == less_or_equal) {
        // expr <= 0
        rhs = 0.0;
      }
      else if (constraint._operator == less_than) {
        // expr < 0  =>  expr <= -epsilon
        rhs = -epsilon_;
      }
      else if (constraint._operator == greater_or_equal) {
        // expr >= 0
        lhs = 0.0;
      }
      else if (constraint._operator == greater_than) {
        // expr > 0  =>  expr >= epsilon
        lhs = epsilon_;
      }
      else if (constraint._operator == equal) {
        // expr == 0
        lhs = rhs = 0.0;
      }
      else if (constraint._operator == not_equal) {
        // x != y  =>  |x - y| >= epsilon
        // Use SCIP's built-in absolute value expression
        SCIP_EXPR* absExpr;
        SCIPcreateExprAbs(scip_, &absExpr, diffExpr, nullptr, nullptr);

        SCIP_CONS* absGeCons;
        SCIPcreateConsBasicNonlinear(scip_, &absGeCons, consName.c_str(), absExpr,
                      epsilon_, SCIPinfinity(scip_));
        SCIPaddCons(scip_, absGeCons);
        SCIPreleaseCons(scip_, &absGeCons);
        SCIPreleaseExpr(scip_, &absExpr);
        SCIPreleaseExpr(scip_, &diffExpr);

        continue;
      }

      // Create nonlinear constraint
      SCIP_CONS* cons;
      SCIPcreateConsBasicNonlinear(scip_, &cons, consName.c_str(), diffExpr, lhs, rhs);
      SCIPaddCons(scip_, cons);
      SCIPreleaseCons(scip_, &cons);
      SCIPreleaseExpr(scip_, &diffExpr);
    }
  }
}

SCIP_EXPR* SCIPSolver::addIndexingConstraints(const std::string& name, const std::vector<SCIP_VAR*>& arrayVars, SCIP_VAR* indexVar, SCIP_VAR* resultVar) {
  size_t n = arrayVars.size();
  int indexOffset = 0;

  // Create binary variables b[i] for each position i
  std::vector<SCIP_VAR*> binaries(n);
  for (size_t i = 0; i < n; i++) {
    std::string binName = name + "_b[" + std::to_string(i) + "]";
    SCIP_VAR* binVar;
    SCIPcreateVarBasic(scip_, &binVar, binName.c_str(), 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
    SCIPaddVar(scip_, binVar);
    binaries[i] = binVar;
  }

  // Constraint: sum(b[i]) = 1 (exactly one position selected)
  SCIP_CONS* sumCons;
  std::vector<double> ones(n, 1.0);
  std::string sumName = name + "_sum";
  SCIPcreateConsBasicLinear(scip_, &sumCons, sumName.c_str(), n, binaries.data(), ones.data(), 1.0, 1.0);
  SCIPaddCons(scip_, sumCons);
  SCIPreleaseCons(scip_, &sumCons);

  // Constraint: index = sum((i+offset) * b[i])
  SCIP_CONS* indexCons;
  std::vector<SCIP_VAR*> indexVars;
  std::vector<double> indexCoeffs;

  indexVars.push_back(indexVar);
  indexCoeffs.push_back(-1.0);

  for (size_t i = 0; i < n; i++) {
    indexVars.push_back(binaries[i]);
    indexCoeffs.push_back(static_cast<double>(i + indexOffset));
  }

  std::string indexName = name + "_index";
  SCIPcreateConsBasicLinear(scip_, &indexCons, indexName.c_str(), indexVars.size(), indexVars.data(), indexCoeffs.data(), -epsilon_, epsilon_);
  SCIPaddCons(scip_, indexCons);
  SCIPreleaseCons(scip_, &indexCons);

  // Constraint: result = sum(b[i] * array[i])
  // This is algebraic (no big-M), consistent with if-then-else implementation
  std::vector<SCIP_EXPR*> productExprs;

  for (size_t i = 0; i < n; i++) {
    // Create b[i] * array[i]
    SCIP_EXPR* binExpr;
    SCIPcreateExprVar(scip_, &binExpr, binaries[i], nullptr, nullptr);

    SCIP_EXPR* arrayExpr;
    SCIPcreateExprVar(scip_, &arrayExpr, arrayVars[i], nullptr, nullptr);

    SCIP_EXPR* productExpr;
    SCIP_EXPR* factors[] = { binExpr, arrayExpr };
    SCIPcreateExprProduct(scip_, &productExpr, 2, factors, 1.0, nullptr, nullptr);

    productExprs.push_back(productExpr);

    SCIPreleaseExpr(scip_, &binExpr);
    SCIPreleaseExpr(scip_, &arrayExpr);
  }

  // Create sum of products
  SCIP_EXPR* sumExpr;
  std::vector<double> coeffs(productExprs.size(), 1.0);
  SCIPcreateExprSum(scip_, &sumExpr, productExprs.size(), productExprs.data(),
      coeffs.data(), 0.0, nullptr, nullptr);

  // Create result variable expression
  SCIP_EXPR* resultExpr_tmp;
  SCIPcreateExprVar(scip_, &resultExpr_tmp, resultVar, nullptr, nullptr);

  // Constraint: result - sum(b[i] * array[i]) = 0
  SCIP_EXPR* diffExpr;
  SCIP_EXPR* diffChildren[] = { resultExpr_tmp, sumExpr };
  double diffCoeffs[] = { 1.0, -1.0 };
  SCIPcreateExprSum(scip_, &diffExpr, 2, diffChildren, diffCoeffs, 0.0, nullptr, nullptr);

  SCIP_CONS* resultCons;
  std::string resultConsName = name + "_result";
  SCIPcreateConsBasicNonlinear(scip_, &resultCons, resultConsName.c_str(), diffExpr, 0.0, 0.0);
  SCIPaddCons(scip_, resultCons);
  SCIPreleaseCons(scip_, &resultCons);

  // Release expressions
  SCIPreleaseExpr(scip_, &diffExpr);
  SCIPreleaseExpr(scip_, &resultExpr_tmp);
  SCIPreleaseExpr(scip_, &sumExpr);
  for (auto expr : productExprs) {
    SCIPreleaseExpr(scip_, &expr);
  }

  // Release binary variables
  for (size_t i = 0; i < n; i++) {
    SCIPreleaseVar(scip_, &binaries[i]);
  }

  // Return expression for result variable
  SCIP_EXPR* resultExpr;
  SCIPcreateExprVar(scip_, &resultExpr, resultVar, nullptr, nullptr);
  return resultExpr;
}

std::expected<Solution, std::string> SCIPSolver::solve(const Model& model) {
  // Solve the problem
  SCIP_RETCODE retcode = SCIPsolve(scip_);
  if (retcode != SCIP_OKAY) {
    return std::unexpected("SCIP solve failed");
  }

  // Get the best solution
  SCIP_SOL* sol = SCIPgetBestSol(scip_);
  if (!sol) {
    return std::unexpected("No solution found");
  }

  // Create CP solution object
  Solution solution(model);

  // Extract variable values from SCIP solution
  for (const auto& [cpVar, scipVar] : variableMap_) {
    double value = SCIPgetSolVal(scip_, sol, scipVar);
    solution.setVariableValue(*cpVar, value);
  }

  return solution;
}

} // namespace CP
