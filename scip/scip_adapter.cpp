#include "scip_adapter.h"
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include <scip/expr_abs.h>
#include <scip/cons_linear.h>
#include <scip/cons_nonlinear.h>
#include <scip/expr_var.h>
#include <scip/expr_product.h>
#include <scip/expr_sum.h>
#include <scip/type_message.h>
#include <limits>
#include <iostream>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <unordered_set>

namespace CP {

// Helper to extract constant value from operand (handles direct double or Expression(none, {double}))
static std::optional<double> extractConstant(const Operand& operand) {
  if (std::holds_alternative<double>(operand)) {
    return std::get<double>(operand);
  } else if (std::holds_alternative<Expression>(operand)) {
    const Expression& expr = std::get<Expression>(operand);
    if (expr._operator == Expression::Operator::none && expr.operands.size() == 1) {
      if (std::holds_alternative<double>(expr.operands[0])) {
        return std::get<double>(expr.operands[0]);
      }
    }
  }
  return std::nullopt;
}

SCIPSolver::SCIPSolver(const Model& model, unsigned int precision)
  : precision(precision)
{
  SCIPcreate(&scip);
  SCIPincludeDefaultPlugins(scip);
  SCIPcreateProbBasic(scip, "cp_model");

  // Set verbosity to errors and warnings only
  SCIPsetIntParam(scip, "display/verblevel", SCIP_VERBLEVEL_NONE);

  // Query SCIP's feasibility tolerance for constraint formulations
  // This ensures our epsilon-based constraints use SCIP's feasibility tolerance
  SCIPgetRealParam(scip, "numerics/feastol", &epsilon);

  addSequences(model);
  addVariables(model);
  addIndexedVariables(model);
  addDeducedConstraints(model);
  addObjective(model);
  addConstraints(model);
}

SCIPSolver::~SCIPSolver() {
  if (scip) {
    // Release all variables
    for (auto& [variable, scipVar] : variableMap) {
      SCIPreleaseVar(scip, &scipVar);
    }
    SCIPfree(&scip);
  }
}

void SCIPSolver::addSequences(const Model& model) {
  for (const auto& sequence : model.getSequences()) {
    // Create sequence variables and collect SCIP vars
    std::vector<SCIP_VAR*> sequenceVariables;
    for (const Variable& variable : sequence.variables) {

      SCIP_VAR* scipVar;
      SCIPcreateVarBasic(scip, &scipVar, variable.name.c_str(), 1, sequence.variables.size(), 0.0, SCIP_VARTYPE_INTEGER);
      SCIPaddVar(scip, scipVar);
      variableMap[&variable] = scipVar;
      sequenceVariables.push_back(scipVar);
    }

    // Add sequence constraint (alldifferent permutation of {1, ..., n})
    addSequenceConstraints(sequence.name, sequenceVariables);
  }
}

void SCIPSolver::addSequenceConstraints(const std::string& sequenceName, const std::vector<SCIP_VAR*>& sequenceVariables) {
  size_t n = sequenceVariables.size();

  // Binary matrix formulation for alldifferent
  // Create n×n binary variables b[i][v] for each position i and value v in {1, ..., n}
  std::vector<std::vector<SCIP_VAR*>> binaries(n);
  for (size_t i = 0; i < n; i++) {
    binaries[i].resize(n);
    for (int value = 1; value <= static_cast<int>(n); value++) {
      std::string binaryName = sequenceName + "_b[" + std::to_string(i) + "][" + std::to_string(value) + "]";
      SCIP_VAR* binaryVar;
      SCIPcreateVarBasic(scip, &binaryVar, binaryName.c_str(), 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
      SCIPaddVar(scip, binaryVar);
      binaries[i][value - 1] = binaryVar;
    }
  }

  // Row constraints: sum_v b[i][v] = 1 for each position i
  for (size_t i = 0; i < n; i++) {
    SCIP_CONS* rowCons;
    std::vector<double> coeffs(binaries[i].size(), 1.0);
    std::string consName = sequenceName + "_row[" + std::to_string(i) + "]";
    SCIPcreateConsBasicLinear(scip, &rowCons, consName.c_str(), binaries[i].size(), binaries[i].data(), coeffs.data(), 1.0, 1.0);
    SCIPaddCons(scip, rowCons);
    SCIPreleaseCons(scip, &rowCons);
  }

  // Column constraints: sum_i b[i][v] = 1 for each value v
  for (int value = 1; value <= static_cast<int>(n); value++) {
    SCIP_CONS* colCons;
    std::vector<SCIP_VAR*> colVars(n);
    std::vector<double> coeffs(n, 1.0);
    for (size_t i = 0; i < n; i++) {
      colVars[i] = binaries[i][value - 1];
    }
    std::string consName = sequenceName + "_col[" + std::to_string(value) + "]";
    SCIPcreateConsBasicLinear(scip, &colCons, consName.c_str(), n, colVars.data(), coeffs.data(), 1.0, 1.0);
    SCIPaddCons(scip, colCons);
    SCIPreleaseCons(scip, &colCons);
  }

  // Link sequence variable with binary matrix: x[i] = sum_v (v * b[i][v])
  for (size_t i = 0; i < n; i++) {
    SCIP_VAR* scipVar = sequenceVariables[i];

    SCIP_CONS* linkCons;
    std::vector<SCIP_VAR*> linkVars;
    std::vector<double> linkCoeffs;

    // Add x[i] with coefficient -1
    linkVars.push_back(scipVar);
    linkCoeffs.push_back(-1.0);

    // Add b[i][v] with coefficient v
    for (int value = 1; value <= static_cast<int>(n); value++) {
      linkVars.push_back(binaries[i][value - 1]);
      linkCoeffs.push_back(static_cast<double>(value));
    }

    std::string consName = sequenceName + "_link[" + std::to_string(i) + "]";
    SCIPcreateConsBasicLinear(scip, &linkCons, consName.c_str(), linkVars.size(), linkVars.data(), linkCoeffs.data(), -epsilon, epsilon);
    SCIPaddCons(scip, linkCons);
    SCIPreleaseCons(scip, &linkCons);
  }

  // Release binary variables
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < binaries[i].size(); j++) {
      SCIPreleaseVar(scip, &binaries[i][j]);
    }
  }
}

void SCIPSolver::addVariables(const Model& model) {
  for (const auto& variable : model.getVariables()) {
    SCIP_VARTYPE vartype;

    switch (variable.type) {
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
    double lowerBound = (variable.lowerBound == std::numeric_limits<double>::lowest())
                        ? -SCIPinfinity(scip) : variable.lowerBound;
    double upperBound = (variable.upperBound == std::numeric_limits<double>::max())
                        ? SCIPinfinity(scip) : variable.upperBound;

    SCIP_VAR* scipVar;
    SCIPcreateVarBasic(scip, &scipVar, variable.name.c_str(), lowerBound, upperBound, 0.0, vartype);
    SCIPaddVar(scip, scipVar);
    variableMap[&variable] = scipVar;
  }
}

void SCIPSolver::addIndexedVariables(const Model& model) {
  for (const auto& indexedVariables : model.getIndexedVariables()) {
    for (const auto& indexedVariable : indexedVariables) {
      SCIP_VARTYPE vartype;

      switch (indexedVariable.type) {
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
      double lowerBound = (indexedVariable.lowerBound == std::numeric_limits<double>::lowest())
                          ? -SCIPinfinity(scip) : indexedVariable.lowerBound;
      double upperBound = (indexedVariable.upperBound == std::numeric_limits<double>::max())
                          ? SCIPinfinity(scip) : indexedVariable.upperBound;

      SCIP_VAR* scipVar;
      SCIPcreateVarBasic(scip, &scipVar, indexedVariable.name.c_str(), lowerBound, upperBound, 0.0, vartype);
      SCIPaddVar(scip, scipVar);
      variableMap[&indexedVariable] = scipVar;
    }
  }
}

SCIP_EXPR* SCIPSolver::boolify(SCIP_EXPR* scipExpr) {
  // Convert expression to boolean: 0 if abs(scipExpr) <= epsilon, 1 if abs(scipExpr) >= 1.1 * epsilon
  // Using epsilon-based algebraic formulation (no big-M)

  // Create binary variable b
  size_t auxId = auxiliaryCounter++;
  std::string boolName = "bool_aux_" + std::to_string(auxId);
  SCIP_VAR* boolVar;
  SCIPcreateVarBasic(scip, &boolVar, boolName.c_str(), 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
  SCIPaddVar(scip, boolVar);

  // Create abs(scipExpr)
  SCIP_EXPR* absExpr;
  SCIPcreateExprAbs(scip, &absExpr, scipExpr, nullptr, nullptr);

  // Constraint 1: abs(scipExpr) >= 1.1 * epsilon * b
  // If b=1, then abs(scipExpr) >= 1.1 * epsilon (strict side with tolerance)
  SCIP_EXPR* boolVarExpr1;
  SCIPcreateExprVar(scip, &boolVarExpr1, boolVar, nullptr, nullptr);

  SCIP_EXPR* epsilonB;
  double coeff1 = 1.1 * epsilon;
  SCIPcreateExprSum(scip, &epsilonB, 1, &boolVarExpr1, &coeff1, 0.0, nullptr, nullptr);

  SCIP_EXPR* diff1;
  SCIP_EXPR* children1[] = { absExpr, epsilonB };
  double coeffs1[] = { 1.0, -1.0 };
  SCIPcreateExprSum(scip, &diff1, 2, children1, coeffs1, 0.0, nullptr, nullptr);

  std::string lowerConsName = "bool_lower_" + std::to_string(auxId);
  SCIP_CONS* cons1;
  SCIPcreateConsBasicNonlinear(scip, &cons1, lowerConsName.c_str(), diff1, 0.0, SCIPinfinity(scip));
  SCIPaddCons(scip, cons1);
  SCIPreleaseCons(scip, &cons1);
  SCIPreleaseExpr(scip, &diff1);
  SCIPreleaseExpr(scip, &epsilonB);
  SCIPreleaseExpr(scip, &boolVarExpr1);

  // Constraint 2: (1-b) * (abs(scipExpr) - epsilon) <= 0
  // If b=0, then abs(scipExpr) <= epsilon (forces small values when b=0)
  // If b=1, constraint is 0 <= 0 (always satisfied)
  SCIP_EXPR* absExpr2;
  SCIPduplicateExpr(scip, absExpr, &absExpr2, nullptr, nullptr, nullptr, nullptr);

  SCIP_EXPR* boolVarExpr2;
  SCIPcreateExprVar(scip, &boolVarExpr2, boolVar, nullptr, nullptr);

  // (1 - b)
  SCIP_EXPR* oneMinusB;
  double coeff2 = -1.0;
  SCIPcreateExprSum(scip, &oneMinusB, 1, &boolVarExpr2, &coeff2, 1.0, nullptr, nullptr);

  // (abs(scipExpr) - epsilon)
  SCIP_EXPR* absMinusEps;
  SCIPcreateExprSum(scip, &absMinusEps, 1, &absExpr2, nullptr, -epsilon, nullptr, nullptr);

  // (1-b) * (abs(scipExpr) - epsilon)
  SCIP_EXPR* product;
  SCIP_EXPR* prodChildren[] = { oneMinusB, absMinusEps };
  SCIPcreateExprProduct(scip, &product, 2, prodChildren, 1.0, nullptr, nullptr);

  // (1-b) * (abs(scipExpr) - epsilon) <= 0
  std::string upperConsName = "bool_upper_" + std::to_string(auxId);
  SCIP_CONS* cons2;
  SCIPcreateConsBasicNonlinear(scip, &cons2, upperConsName.c_str(), product, -SCIPinfinity(scip), 0.0);
  SCIPaddCons(scip, cons2);
  SCIPreleaseCons(scip, &cons2);
  SCIPreleaseExpr(scip, &product);
  SCIPreleaseExpr(scip, &absMinusEps);
  SCIPreleaseExpr(scip, &oneMinusB);
  SCIPreleaseExpr(scip, &boolVarExpr2);
  SCIPreleaseExpr(scip, &absExpr2);
  SCIPreleaseExpr(scip, &absExpr);

  // Return expression for boolean variable
  SCIP_EXPR* resultExpr;
  SCIPcreateExprVar(scip, &resultExpr, boolVar, nullptr, nullptr);
  SCIPreleaseVar(scip, &boolVar);

  return resultExpr;
}

SCIP_EXPR* SCIPSolver::buildExpression(const Model& model, const Operand& operand) {
  // Handle constants
  if (std::holds_alternative<double>(operand)) {
    SCIP_EXPR* scipExpr;
    SCIPcreateExprValue(scip, &scipExpr, std::get<double>(operand), nullptr, nullptr);
    return scipExpr;
  }

  // Handle variables
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(operand)) {
    const Variable& var = std::get<std::reference_wrapper<const Variable>>(operand).get();
    auto it = variableMap.find(&var);
    if (it == variableMap.end()) {
      throw std::runtime_error("SCIPSolver: Variable not found in variableMap: " + var.name);
    }
    SCIP_EXPR* scipExpr;
    SCIPcreateExprVar(scip, &scipExpr, it->second, nullptr, nullptr);
    return scipExpr;
  }

  // Handle indexed variables (element constraint: result = array[index])
  if (std::holds_alternative<IndexedVariable>(operand)) {
    const IndexedVariable& indexedVar = std::get<IndexedVariable>(operand);
    const IndexedVariables& container = indexedVar.container.get();
    const Variable& indexVar = indexedVar.index.get();

    // Get index SCIP variable
    auto indexIt = variableMap.find(&indexVar);
    if (indexIt == variableMap.end()) {
      throw std::runtime_error("SCIPSolver: Index variable not found in variableMap: " + indexVar.name);
    }
    SCIP_VAR* scipIndexVar = indexIt->second;

    // Get array SCIP variables
    std::vector<SCIP_VAR*> arrayVars;
    for (const auto& var : container) {
      auto varIt = variableMap.find(&var);
      if (varIt == variableMap.end()) {
        throw std::runtime_error("SCIPSolver: Array variable not found in variableMap: " + var.name);
      }
      arrayVars.push_back(varIt->second);
    }

    // Create result variable (unbounded - let constraint determine valid range)
    size_t auxId = auxiliaryCounter++;
    SCIP_VAR* resultVar;
    std::string resultName = container.name + "[" + indexVar.name + "]_result_" + std::to_string(auxId);
    SCIPcreateVarBasic(scip, &resultVar, resultName.c_str(), -SCIPinfinity(scip), SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS);
    SCIPaddVar(scip, resultVar);

    // Add element constraint (0-based indexing)
    std::string elemName = container.name + "[" + indexVar.name + "]_elem_" + std::to_string(auxId);
    SCIP_EXPR* resultExpr = addIndexingConstraints(elemName, arrayVars, scipIndexVar, resultVar);

    SCIPreleaseVar(scip, &resultVar);
    return resultExpr;
  }

  // Handle expressions
  if (std::holds_alternative<Expression>(operand)) {
    const Expression& expression = std::get<Expression>(operand);
    using enum Expression::Operator;

    // Base case: none operator
    if (expression._operator == none && expression.operands.size() == 1) {
      return buildExpression(model, expression.operands[0]);
    }

    // Negate: -scipExpr
    if (expression._operator == negate && expression.operands.size() == 1) {
      auto subExpression = buildExpression(model, expression.operands[0]);

      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { subExpression };
      double coeffs[] = { -1.0 };
      SCIPcreateExprSum(scip, &scipExpr, 1, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &children[0]);
      return scipExpr;
    }

    // Addition: a + b
    if (expression._operator == add && expression.operands.size() == 2) {
      auto left = buildExpression(model, expression.operands[0]);
      auto right = buildExpression(model, expression.operands[1]);

      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { left, right };
      double coeffs[] = { 1.0, 1.0 };
      SCIPcreateExprSum(scip, &scipExpr, 2, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &children[0]);
      SCIPreleaseExpr(scip, &children[1]);
      return scipExpr;
    }

    // Subtraction: a - b
    if (expression._operator == subtract && expression.operands.size() == 2) {
      auto left = buildExpression(model, expression.operands[0]);
      auto right = buildExpression(model, expression.operands[1]);

      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { left, right };
      double coeffs[] = { 1.0, -1.0 };
      SCIPcreateExprSum(scip, &scipExpr, 2, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &children[0]);
      SCIPreleaseExpr(scip, &children[1]);
      return scipExpr;
    }

    // Multiplication: a * b
    if (expression._operator == multiply && expression.operands.size() == 2) {
      auto left = buildExpression(model, expression.operands[0]);
      auto right = buildExpression(model, expression.operands[1]);

      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { left, right };
      SCIPcreateExprProduct(scip, &scipExpr, 2, children, 1.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &children[0]);
      SCIPreleaseExpr(scip, &children[1]);
      return scipExpr;
    }

    // Division: a / b
    if (expression._operator == divide && expression.operands.size() == 2) {
      auto numerator = buildExpression(model, expression.operands[0]);
      auto denominator = buildExpression(model, expression.operands[1]);

      // Division is a * b^(-1)
      SCIP_EXPR* powExpr;
      SCIPcreateExprPow(scip, &powExpr, denominator, -1.0, nullptr, nullptr);

      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { numerator, powExpr };
      SCIPcreateExprProduct(scip, &scipExpr, 2, children, 1.0, nullptr, nullptr);

      SCIPreleaseExpr(scip, &children[0]);
      SCIPreleaseExpr(scip, &children[1]);
      SCIPreleaseExpr(scip, &denominator);
      return scipExpr;
    }

    // Logical NOT: !a  => 1 - bool(a)
    if (expression._operator == logical_not && expression.operands.size() == 1) {
      auto subExpression = buildExpression(model, expression.operands[0]);

      // Convert to boolean first
      SCIP_EXPR* boolExpr = boolify(subExpression);
      SCIPreleaseExpr(scip, &subExpression);

      // NOT: 1 - bool(a)
      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { boolExpr };
      double coeffs[] = { -1.0 };
      SCIPcreateExprSum(scip, &scipExpr, 1, children, coeffs, 1.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &boolExpr);
      return scipExpr;
    }

    // Logical AND: a && b  => bool(a) * bool(b)
    if (expression._operator == logical_and && expression.operands.size() == 2) {
      auto left = buildExpression(model, expression.operands[0]);
      auto right = buildExpression(model, expression.operands[1]);

      // Convert to booleans first
      SCIP_EXPR* boolLeft = boolify(left);
      SCIP_EXPR* boolRight = boolify(right);
      SCIPreleaseExpr(scip, &left);
      SCIPreleaseExpr(scip, &right);

      // AND: bool(a) * bool(b)
      SCIP_EXPR* scipExpr;
      SCIP_EXPR* children[] = { boolLeft, boolRight };
      SCIPcreateExprProduct(scip, &scipExpr, 2, children, 1.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &boolLeft);
      SCIPreleaseExpr(scip, &boolRight);
      return scipExpr;
    }

    // Logical OR: a || b  => bool(bool(a) + bool(b))
    if (expression._operator == logical_or && expression.operands.size() == 2) {
      auto left = buildExpression(model, expression.operands[0]);
      auto right = buildExpression(model, expression.operands[1]);

      // Convert to booleans first
      SCIP_EXPR* boolLeft = boolify(left);
      SCIP_EXPR* boolRight = boolify(right);
      SCIPreleaseExpr(scip, &left);
      SCIPreleaseExpr(scip, &right);

      // OR: bool(a) + bool(b)
      SCIP_EXPR* sumExpr;
      SCIP_EXPR* sumChildren[] = { boolLeft, boolRight };
      double coeffs[] = { 1.0, 1.0 };
      SCIPcreateExprSum(scip, &sumExpr, 2, sumChildren, coeffs, 0.0, nullptr, nullptr);

      SCIPreleaseExpr(scip, &boolLeft);
      SCIPreleaseExpr(scip, &boolRight);

      // Convert sum back to boolean
      SCIP_EXPR* scipExpr = boolify(sumExpr);
      SCIPreleaseExpr(scip, &sumExpr);
      return scipExpr;
    }

    // Collection: wrapper for collections, just unwrap
    if (expression._operator == collection && expression.operands.size() == 1) {
      return buildExpression(model, expression.operands[0]);
    }

    // At: indexed access using [] operator - collection(key)[index]
    if (expression._operator == at && expression.operands.size() == 2) {
      // operands[0] = collection expression
      // operands[1] = index expression

      if (!std::holds_alternative<Expression>(expression.operands[0])) {
        throw std::runtime_error("SCIPSolver: First operand must be an expression");
      }

      const Expression& collectionExpr = std::get<Expression>(expression.operands[0]);

      // Check if this is collection(key)[index] syntax
      if (collectionExpr._operator == collection && collectionExpr.operands.size() == 1) {
        // This is collection(key)[index] - delegate to the same implementation as at(index, collection(key))
        // We need to create an equivalent custom operator expression structure

        // Extract key from collection(key) - can be variable or constant
        const Operand& keyOperand = collectionExpr.operands[0];

        // Handle constant key
        auto constantKeyOpt = extractConstant(keyOperand);
        if (constantKeyOpt.has_value()) {
          double constantKey = constantKeyOpt.value();
          auto collectionResult = model.getCollection(constantKey);
          if (!collectionResult) {
            throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(constantKey) +
                                     " not found in model");
          }

          const std::vector<double>& collection = collectionResult.value();
          const Operand& indexOperand = expression.operands[1];

          // With constant key, just need to handle the index
          auto constantIndexOpt = extractConstant(indexOperand);
          if (constantIndexOpt.has_value()) {
            size_t index = (size_t)constantIndexOpt.value();
            if (index < 1 || index > collection.size()) {
              throw std::runtime_error("SCIPSolver: Index out of bounds");
            }
            // Return constant value
            SCIP_EXPR* resultExpr;
            double value = collection[index - 1];
            SCIPcreateExprValue(scip, &resultExpr, value, nullptr, nullptr);
            return resultExpr;
          } else {
            // Variable index with constant key - use 1D element constraint
            SCIP_VAR* scipIndexVar = nullptr;
            if (std::holds_alternative<std::reference_wrapper<const Variable>>(indexOperand)) {
              const Variable& indexVar = std::get<std::reference_wrapper<const Variable>>(indexOperand).get();
              scipIndexVar = variableMap.at(&indexVar);
            } else {
              throw std::runtime_error("SCIPSolver: Index must be a variable or constant");
            }

            double indexLb = SCIPvarGetLbGlobal(scipIndexVar);
            std::vector<double> elements(collection.begin(), collection.end());
            return buildElementConstraint(elements, scipIndexVar, indexLb);
          }
        }

        // Handle variable key
        if (!std::holds_alternative<std::reference_wrapper<const Variable>>(keyOperand)) {
          throw std::runtime_error(
            "SCIPSolver: collection() key must be a variable or constant"
          );
        }

        const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(keyOperand).get();
        SCIP_VAR* scipKeyVar = variableMap.at(&keyVar);

        const Operand& indexOperand = expression.operands[1];

        // Get key bounds
        double keyLb = SCIPvarGetLbGlobal(scipKeyVar);
        double keyUb = SCIPvarGetUbGlobal(scipKeyVar);
        int numKeys = (int)std::floor(keyUb) - (int)std::ceil(keyLb) + 1;

        // Case A: Index is a constant
        if (std::holds_alternative<double>(indexOperand)) {
          size_t index = (size_t)std::get<double>(indexOperand);

          if (index < 1) {
            throw std::runtime_error("SCIPSolver: Index must be >= 1 (1-based indexing)");
          }

          // Pre-compute element at this index for each collection
          std::vector<double> elements(numKeys);

          for (int i = 0; i < numKeys; i++) {
            int key = (int)std::ceil(keyLb) + i;
            auto collectionResult = model.getCollection((double)key);
            if (!collectionResult) {
              throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                                       " not found in model");
            }

            const std::vector<double>& collection = collectionResult.value();

            if (index > collection.size()) {
              throw std::runtime_error(
                "SCIPSolver: Index " + std::to_string(index) +
                " out of bounds for collection at key " + std::to_string(key) +
                " (size: " + std::to_string(collection.size()) + ")"
              );
            }

            elements[i] = collection[index - 1];  // Convert to 0-based
          }

          // Use 1D element constraint
          return buildElementConstraint(elements, scipKeyVar, keyLb);
        }

        // Case B: Index is a variable - requires 2D matrix
        SCIP_VAR* scipIndexVar = nullptr;

        // Build index expression to extract variable
        auto indexExprResult = buildExpression(model, indexOperand);

        // Try to extract SCIP variable from the index
        if (std::holds_alternative<std::reference_wrapper<const Variable>>(indexOperand)) {
          const Variable& indexVar = std::get<std::reference_wrapper<const Variable>>(indexOperand).get();
          scipIndexVar = variableMap.at(&indexVar);
        }
        else if (std::holds_alternative<Expression>(indexOperand)) {
          // Navigate through 'none' operators to find the underlying Variable
          const Operand* currentOperand = &indexOperand;
          while (std::holds_alternative<Expression>(*currentOperand)) {
            const Expression& expr = std::get<Expression>(*currentOperand);
            if (expr._operator == Expression::Operator::none && expr.operands.size() == 1) {
              currentOperand = &expr.operands[0];
            } else {
              // Complex expression - not a simple variable
              SCIPreleaseExpr(scip, &indexExprResult);
              throw std::runtime_error(
                "SCIPSolver: collection[index] requires index to be a variable or constant"
              );
            }
          }

          if (!std::holds_alternative<std::reference_wrapper<const Variable>>(*currentOperand)) {
            SCIPreleaseExpr(scip, &indexExprResult);
            throw std::runtime_error("SCIPSolver: Failed to extract variable from index");
          }

          const Variable& indexVar = std::get<std::reference_wrapper<const Variable>>(*currentOperand).get();
          scipIndexVar = variableMap.at(&indexVar);
        }
        else {
          SCIPreleaseExpr(scip, &indexExprResult);
          throw std::runtime_error("SCIPSolver: Invalid index operand for collection[]");
        }

        SCIPreleaseExpr(scip, &indexExprResult);

        // Get index bounds (1-based)
        double indexLb = SCIPvarGetLbGlobal(scipIndexVar);
        double indexUb = SCIPvarGetUbGlobal(scipIndexVar);
        int numIndices = (int)std::floor(indexUb) - (int)std::ceil(indexLb) + 1;

        // Build 2D matrix of elements (flattened to 1D)
        std::vector<double> elements2D(numKeys * numIndices);

        for (int ki = 0; ki < numKeys; ki++) {
          int key = (int)std::ceil(keyLb) + ki;
          auto collectionResult = model.getCollection((double)key);
          if (!collectionResult) {
            throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                                     " not found in model");
          }
          const std::vector<double>& collection = collectionResult.value();

          for (int ii = 0; ii < numIndices; ii++) {
            size_t index = (size_t)std::ceil(indexLb) + ii;  // 1-based

            if (index < 1 || index > collection.size()) {
              throw std::runtime_error(
                "SCIPSolver: Index " + std::to_string(index) +
                " out of bounds for collection at key " + std::to_string(key)
              );
            }

            int flatIndex = ki * numIndices + ii;
            elements2D[flatIndex] = collection[index - 1];  // Convert to 0-based
          }
        }

        // Create computed index: (key - keyLb) * numIndices + (index - indexLb)
        SCIP_EXPR* keyExpr;
        SCIPcreateExprVar(scip, &keyExpr, scipKeyVar, nullptr, nullptr);

        SCIP_EXPR* indexExpr;
        SCIPcreateExprVar(scip, &indexExpr, scipIndexVar, nullptr, nullptr);

        // (key - keyLb)
        SCIP_EXPR* keyMinusOffset;
        SCIPcreateExprSum(scip, &keyMinusOffset, 1, &keyExpr, nullptr, -keyLb, nullptr, nullptr);

        // (key - keyLb) * numIndices
        SCIP_EXPR* keyTimesNumIndices;
        double keyCoeff = (double)numIndices;
        SCIPcreateExprSum(scip, &keyTimesNumIndices, 1, &keyMinusOffset, &keyCoeff, 0.0, nullptr, nullptr);

        // (index - indexLb)
        SCIP_EXPR* indexMinusOffset;
        SCIPcreateExprSum(scip, &indexMinusOffset, 1, &indexExpr, nullptr, -indexLb, nullptr, nullptr);

        // (key - keyLb) * numIndices + (index - indexLb)
        SCIP_EXPR* exprs[2] = {keyTimesNumIndices, indexMinusOffset};
        SCIP_EXPR* computedIndexExpr;
        SCIPcreateExprSum(scip, &computedIndexExpr, 2, exprs, nullptr, 0.0, nullptr, nullptr);

        // Convert to integer variable for the computed index
        SCIP_VAR* computedIndexVar;
        std::string indexName = "computed_index_" + std::to_string(auxiliaryCounter++);
        SCIPcreateVarBasic(scip, &computedIndexVar, indexName.c_str(),
          0.0, (double)(numKeys * numIndices - 1), 0.0, SCIP_VARTYPE_INTEGER);
        SCIPaddVar(scip, computedIndexVar);

        // Add constraint: computedIndexVar == computedIndexExpr
        SCIP_CONS* indexCons;
        SCIP_EXPR* indexVarExpr;
        SCIPcreateExprVar(scip, &indexVarExpr, computedIndexVar, nullptr, nullptr);

        SCIP_EXPR* indexDiff;
        SCIP_EXPR* exprsForDiff[2] = {indexVarExpr, computedIndexExpr};
        double coeffs[2] = {1.0, -1.0};
        SCIPcreateExprSum(scip, &indexDiff, 2, exprsForDiff, coeffs, 0.0, nullptr, nullptr);

        std::string consName = "index_def_" + std::to_string(auxiliaryCounter++);
        SCIPcreateConsBasicNonlinear(scip, &indexCons, consName.c_str(), indexDiff, 0.0, 0.0);
        SCIPaddCons(scip, indexCons);
        SCIPreleaseCons(scip, &indexCons);

        // Release intermediate expressions
        SCIPreleaseExpr(scip, &keyExpr);
        SCIPreleaseExpr(scip, &indexExpr);
        SCIPreleaseExpr(scip, &keyMinusOffset);
        SCIPreleaseExpr(scip, &keyTimesNumIndices);
        SCIPreleaseExpr(scip, &indexMinusOffset);
        SCIPreleaseExpr(scip, &computedIndexExpr);
        SCIPreleaseExpr(scip, &indexVarExpr);
        SCIPreleaseExpr(scip, &indexDiff);

        // Use element constraint with computed index
        SCIP_EXPR* result = buildElementConstraint(elements2D, computedIndexVar, 0.0);
        SCIPreleaseVar(scip, &computedIndexVar);

        return result;
      }

      // Not a collection() expression - might be IndexedVariables or something else
      throw std::runtime_error("SCIPSolver: [] operator only supported for collection() expressions");
    }

    // Collection: should not appear standalone, only as operand to custom operators
    if (expression._operator == collection) {
      throw std::runtime_error(
        "SCIPSolver: collection() expression cannot be used standalone. "
        "It must be used with an operation like count(collection(key))."
      );
    }

    // Custom operators: max, min, sum, pow, etc.
    if (expression._operator == custom && expression.operands.size() >= 2) {
      // First operand is the operator index
      if (!std::holds_alternative<size_t>(expression.operands[0])) {
        throw std::runtime_error("SCIPSolver: Custom operator index missing");
      }
      size_t opIndex = std::get<size_t>(expression.operands[0]);
      std::string opName = Expression::customOperators[opIndex];

      SCIP_EXPR* scipExpr = nullptr;

      // Handle specific custom operators
      if (opName == "pow" && expression.operands.size() == 3) {
        // pow(a, b) - if b is constant
        if (std::holds_alternative<double>(expression.operands[2])) {
          double exponent = std::get<double>(expression.operands[2]);
          auto baseExpr = buildExpression(model, expression.operands[1]);

          SCIPcreateExprPow(scip, &scipExpr, baseExpr, exponent, nullptr, nullptr);
          SCIPreleaseExpr(scip, &baseExpr);
          return scipExpr;
        }
        else {
          throw std::runtime_error("pow with non-constant exponent not supported");
        }
      }

      // Check if any operand is a collection() expression
      bool hasCollection = false;
      for (size_t i = 1; i < expression.operands.size(); i++) {
        if (std::holds_alternative<Expression>(expression.operands[i])) {
          const Expression& operandExpr = std::get<Expression>(expression.operands[i]);
          if (operandExpr._operator == collection) {
            hasCollection = true;
            break;
          }
        }
      }

      // Route collection operations to specialized handler
      if (hasCollection) {
        // Validate that this is a supported collection operation
        static const std::unordered_set<std::string> supportedCollectionOps = {
          "count", "sum", "avg", "max", "min", "element_of", "not_element_of", "at"
        };

        if (supportedCollectionOps.find(opName) == supportedCollectionOps.end()) {
          throw std::runtime_error(
            "SCIPSolver: Custom operator '" + opName + "' cannot take collection() expressions. "
            "Only count, sum, avg, max, min, element_of, not_element_of, and at can process collections."
          );
        }

        // Route to appropriate handler
        if (opName == "count" || opName == "sum" || opName == "avg" ||
            opName == "max" || opName == "min") {
          return resolveCollectionOperation(model, expression, opName);
        }
        else if (opName == "element_of" || opName == "not_element_of") {
          return resolveCollectionMembership(model, expression, opName);
        }
        else if (opName == "at") {
          return resolveCollectionItem(model, expression);
        }
        else {
          throw std::runtime_error(
            "SCIPSolver: Collection operation '" + opName + "' not supported"
          );
        }
      }

      // For other operators, build all child expressions
      std::vector<SCIP_EXPR*> children;
      for (size_t i = 1; i < expression.operands.size(); i++) {
        auto childExpr = buildExpression(model, expression.operands[i]);
        children.push_back(childExpr);
      }

      if (opName == "min" || opName == "max") {
        // min/max(a, b, ...) - use auxiliary variable with constraints
        // Create auxiliary variable for result
        std::string auxName = opName + "_aux_" + std::to_string(auxiliaryCounter++);
        SCIP_VAR* auxVar;
        SCIPcreateVarBasic(scip, &auxVar, auxName.c_str(), -SCIPinfinity(scip), SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS);
        SCIPaddVar(scip, auxVar);

        // For each child, add constraint: auxVar <= child (for min) or auxVar >= child (for max)
        for (auto child : children) {
          SCIP_EXPR* auxExpr;
          SCIPcreateExprVar(scip, &auxExpr, auxVar, nullptr, nullptr);

          // Create constraint: auxVar - child <= 0 (for min) or child - auxVar <= 0 (for max)
          SCIP_EXPR* diffExpr;
          if (opName == "min") {
            // auxVar <= child  =>  auxVar - child <= 0
            SCIP_EXPR* scipExprs[] = { auxExpr, child };
            double coeffs[] = { 1.0, -1.0 };
            SCIPcreateExprSum(scip, &diffExpr, 2, scipExprs, coeffs, 0.0, nullptr, nullptr);
          }
          else { // max
            // auxVar >= child  =>  child - auxVar <= 0
            SCIP_EXPR* scipExprs[] = { child, auxExpr };
            double coeffs[] = { 1.0, -1.0 };
            SCIPcreateExprSum(scip, &diffExpr, 2, scipExprs, coeffs, 0.0, nullptr, nullptr);
          }

          SCIP_CONS* cons;
          SCIPcreateConsBasicNonlinear(scip, &cons, (opName + "_bound").c_str(),
                        diffExpr, -SCIPinfinity(scip), 0.0);
          SCIPaddCons(scip, cons);
          SCIPreleaseCons(scip, &cons);
          SCIPreleaseExpr(scip, &diffExpr);
          SCIPreleaseExpr(scip, &auxExpr);
        }

        // Add disjunctive constraint: auxVar = child_1 OR auxVar = child_2 OR ...
        // This is modeled as: (auxVar - child_1) * (auxVar - child_2) * ... = 0
        std::vector<SCIP_EXPR*> products;
        for (auto child : children) {
          SCIP_EXPR* auxExprCopy;
          SCIPcreateExprVar(scip, &auxExprCopy, auxVar, nullptr, nullptr);

          SCIP_EXPR* diffExpr;
          SCIP_EXPR* scipExprs[] = { auxExprCopy, child };
          double coeffs[] = { 1.0, -1.0 };
          SCIPcreateExprSum(scip, &diffExpr, 2, scipExprs, coeffs, 0.0, nullptr, nullptr);

          products.push_back(diffExpr);
          SCIPreleaseExpr(scip, &auxExprCopy);
        }

        // Create product: (auxVar - child_1) * (auxVar - child_2) * ...
        SCIP_EXPR* productExpr;
        SCIPcreateExprProduct(scip, &productExpr, products.size(), products.data(),
                   1.0, nullptr, nullptr);

        SCIP_CONS* eqCons;
        SCIPcreateConsBasicNonlinear(scip, &eqCons, (opName + "_eq").c_str(),
                      productExpr, 0.0, 0.0);
        SCIPaddCons(scip, eqCons);
        SCIPreleaseCons(scip, &eqCons);
        SCIPreleaseExpr(scip, &productExpr);

        for (auto prod : products) {
          SCIPreleaseExpr(scip, &prod);
        }

        // Create expression for auxiliary variable
        SCIPcreateExprVar(scip, &scipExpr, auxVar, nullptr, nullptr);
        SCIPreleaseVar(scip, &auxVar);

        // Release children
        for (auto child : children) {
          SCIPreleaseExpr(scip, &child);
        }

        return scipExpr;
      }
      else if (opName == "sum") {
        // sum(a, b, ...) = a + b + ...
        std::vector<double> coeffs(children.size(), 1.0);
        SCIPcreateExprSum(scip, &scipExpr, children.size(), children.data(), coeffs.data(), 0.0, nullptr, nullptr);
      }
      else if (opName == "avg") {
        // avg = sum / count
        std::vector<double> coeffs(children.size(), 1.0 / children.size());
        SCIPcreateExprSum(scip, &scipExpr, children.size(), children.data(), coeffs.data(), 0.0, nullptr, nullptr);
      }
      else if (opName == "count") {
        // count(...) returns the number of arguments (constant)
        double count = static_cast<double>(children.size());
        SCIPcreateExprValue(scip, &scipExpr, count, nullptr, nullptr);
      }
      else if (opName == "if_then_else") {
        // if_then_else(c, v1, v2) = c * v1 + (1 - c) * v2
        if (children.size() != 3) {
          for (auto child : children) {
            SCIPreleaseExpr(scip, &child);
          }
          throw std::runtime_error("SCIPSolver: if_then_else requires exactly 3 operands");
        }

        SCIP_EXPR* condition = children[0];
        SCIP_EXPR* ifValue = children[1];
        SCIP_EXPR* elseValue = children[2];

        // Create c * v1
        SCIP_EXPR* term1;
        SCIP_EXPR* factors1[] = { condition, ifValue };
        SCIPcreateExprProduct(scip, &term1, 2, factors1, 1.0, nullptr, nullptr);

        // Create (1 - c)
        SCIP_EXPR* conditionCopy;
        SCIPduplicateExpr(scip, condition, &conditionCopy, nullptr, nullptr, nullptr, nullptr);
        SCIP_EXPR* oneMinusC;
        double coeff = -1.0;
        SCIPcreateExprSum(scip, &oneMinusC, 1, &conditionCopy, &coeff, 1.0, nullptr, nullptr);
        SCIPreleaseExpr(scip, &conditionCopy);

        // Create (1 - c) * v2
        SCIP_EXPR* term2;
        SCIP_EXPR* factors2[] = { oneMinusC, elseValue };
        SCIPcreateExprProduct(scip, &term2, 2, factors2, 1.0, nullptr, nullptr);
        SCIPreleaseExpr(scip, &oneMinusC);

        // Create term1 + term2
        SCIP_EXPR* terms[] = { term1, term2 };
        double coeffs[] = { 1.0, 1.0 };
        SCIPcreateExprSum(scip, &scipExpr, 2, terms, coeffs, 0.0, nullptr, nullptr);

        SCIPreleaseExpr(scip, &term1);
        SCIPreleaseExpr(scip, &term2);
      }
      else if (opName == "at") {
        // at(index, val1, val2, val3, ...) returns val[index]
        // First child is index, remaining children are array values
        if (children.size() < 2) {
          for (auto child : children) {
            SCIPreleaseExpr(scip, &child);
          }
          throw std::runtime_error("SCIPSolver: at requires at least 2 operands (index and at least one value)");
        }

        size_t auxId = auxiliaryCounter++;
        SCIP_EXPR* indexExpr = children[0];
        std::vector<SCIP_EXPR*> arrayExprs(children.begin() + 1, children.end());

        // Create index variable from expression (1-based indexing for custom "at")
        SCIP_VAR* indexVar;
        std::string indexName = "at_index_" + std::to_string(auxId);
        SCIPcreateVarBasic(scip, &indexVar, indexName.c_str(),
                  1, arrayExprs.size(), 0.0, SCIP_VARTYPE_INTEGER);
        SCIPaddVar(scip, indexVar);

        // Constrain index variable to equal the index expression
        SCIP_EXPR* indexVarExpr;
        SCIPcreateExprVar(scip, &indexVarExpr, indexVar, nullptr, nullptr);
        SCIP_EXPR* indexDiff;
        SCIP_EXPR* indexExprs[] = { indexVarExpr, indexExpr };
        double indexCoeffs[] = { 1.0, -1.0 };
        SCIPcreateExprSum(scip, &indexDiff, 2, indexExprs, indexCoeffs, 0.0, nullptr, nullptr);
        SCIP_CONS* indexCons;
        std::string indexConsName = "at_index_eq_" + std::to_string(auxId);
        SCIPcreateConsBasicNonlinear(scip, &indexCons, indexConsName.c_str(), indexDiff, 0.0, 0.0);
        SCIPaddCons(scip, indexCons);
        SCIPreleaseCons(scip, &indexCons);
        SCIPreleaseExpr(scip, &indexDiff);
        SCIPreleaseExpr(scip, &indexVarExpr);

        // Create array variables from expressions (need to materialize expressions as variables)
        std::vector<SCIP_VAR*> arrayVars;
        for (size_t i = 0; i < arrayExprs.size(); i++) {
          SCIP_VAR* arrayVar;
          std::string arrayVarName = "at_array_" + std::to_string(i) + "_" + std::to_string(auxId);
          SCIPcreateVarBasic(scip, &arrayVar, arrayVarName.c_str(), -SCIPinfinity(scip), SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS);
          SCIPaddVar(scip, arrayVar);

          // Constrain array variable to equal the expression
          SCIP_EXPR* arrayVarExpr;
          SCIPcreateExprVar(scip, &arrayVarExpr, arrayVar, nullptr, nullptr);
          SCIP_EXPR* arrayDiff;
          SCIP_EXPR* arrayDiffExprs[] = { arrayVarExpr, arrayExprs[i] };
          double arrayCoeffs[] = { 1.0, -1.0 };
          SCIPcreateExprSum(scip, &arrayDiff, 2, arrayDiffExprs, arrayCoeffs, 0.0, nullptr, nullptr);
          SCIP_CONS* arrayCons;
          std::string arrayConsName = "at_array_eq_" + std::to_string(i) + "_" + std::to_string(auxId);
          SCIPcreateConsBasicNonlinear(scip, &arrayCons, arrayConsName.c_str(), arrayDiff, 0.0, 0.0);
          SCIPaddCons(scip, arrayCons);
          SCIPreleaseCons(scip, &arrayCons);
          SCIPreleaseExpr(scip, &arrayDiff);
          SCIPreleaseExpr(scip, &arrayVarExpr);

          arrayVars.push_back(arrayVar);
        }

        // Create result variable
        SCIP_VAR* resultVar;
        std::string resultName = "at_result_" + std::to_string(auxId);
        SCIPcreateVarBasic(scip, &resultVar, resultName.c_str(),
                  -SCIPinfinity(scip), SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS);
        SCIPaddVar(scip, resultVar);

        // Add element constraint with 1-based indexing for custom "at" operator
        std::string elemName = "at_elem_" + std::to_string(auxId);
        SCIP_EXPR* resultExpr = addIndexingConstraints(elemName, arrayVars, indexVar, resultVar, 1);

        // Release variables
        SCIPreleaseVar(scip, &indexVar);
        for (auto var : arrayVars) {
          SCIPreleaseVar(scip, &var);
        }
        SCIPreleaseVar(scip, &resultVar);

        // Release children
        for (auto child : children) {
          SCIPreleaseExpr(scip, &child);
        }

        return resultExpr;
      }
      else if (opName == "n_ary_if") {
        // n_ary_if(c1, v1, c2, v2, ..., cN, vN, vElse)
        // result = c1*v1 + (1-c1)*c2*v2 + (1-c1)*(1-c2)*c3*v3 + ... + (1-c1)*...*(1-cN)*vElse

        if (children.size() % 2 != 1) {
          for (auto child : children) {
            SCIPreleaseExpr(scip, &child);
          }
          throw std::runtime_error("SCIPSolver: n_ary_if requires an odd number of operands");
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
            SCIPcreateExprProduct(scip, &prefix, 2, factors, 1.0, nullptr, nullptr);
          }
          else {
            // Build (1-c1) * (1-c2) * ... * (1-c_{i-1}) * ci * vi
            std::vector<SCIP_EXPR*> factors;

            // Add (1 - cj) for j < i
            for (size_t j = 0; j < i; j++) {
              SCIP_EXPR* cj = children[2*j];
              SCIP_EXPR* cjCopy;
              SCIPduplicateExpr(scip, cj, &cjCopy, nullptr, nullptr, nullptr, nullptr);

              // Create (1 - cj)
              SCIP_EXPR* oneMinus;
              double coeff = -1.0;
              SCIPcreateExprSum(scip, &oneMinus, 1, &cjCopy, &coeff, 1.0, nullptr, nullptr);
              factors.push_back(oneMinus);
              SCIPreleaseExpr(scip, &cjCopy);
            }

            // Add ci
            factors.push_back(condition);

            // Add vi
            factors.push_back(value);

            // Create product
            SCIPcreateExprProduct(scip, &prefix, factors.size(), factors.data(), 1.0, nullptr, nullptr);

            // Release (1-cj) expressions
            for (size_t j = 0; j < i; j++) {
              SCIPreleaseExpr(scip, &factors[j]);
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
          SCIPduplicateExpr(scip, cj, &cjCopy, nullptr, nullptr, nullptr, nullptr);

          // Create (1 - cj)
          SCIP_EXPR* oneMinus;
          double coeff = -1.0;
          SCIPcreateExprSum(scip, &oneMinus, 1, &cjCopy, &coeff, 1.0, nullptr, nullptr);
          elseFactors.push_back(oneMinus);
          SCIPreleaseExpr(scip, &cjCopy);
        }

        elseFactors.push_back(elseValue);

        SCIP_EXPR* elseTerm;
        SCIPcreateExprProduct(scip, &elseTerm, elseFactors.size(), elseFactors.data(), 1.0, nullptr, nullptr);
        terms.push_back(elseTerm);

        // Release (1-cj) expressions
        for (size_t j = 0; j < numConditions; j++) {
          SCIPreleaseExpr(scip, &elseFactors[j]);
        }

        // Sum all terms
        std::vector<double> coeffs(terms.size(), 1.0);
        SCIPcreateExprSum(scip, &scipExpr, terms.size(), terms.data(), coeffs.data(), 0.0, nullptr, nullptr);

        // Release terms
        for (auto term : terms) {
          SCIPreleaseExpr(scip, &term);
        }
      }
      else {
        // Unsupported custom operator
        for (auto child : children) {
          SCIPreleaseExpr(scip, &child);
        }
        throw std::runtime_error("SCIPSolver: Unsupported custom operator: " + opName);
      }

      // Release children (SCIP keeps its own references)
      for (auto child : children) {
        SCIPreleaseExpr(scip, &child);
      }

      return scipExpr;
    }

    // Handle comparison operators: return binary expression (0 or 1)
    if (expression._operator == less_or_equal ||
        expression._operator == greater_or_equal ||
        expression._operator == equal ||
        expression._operator == less_than ||
        expression._operator == greater_than ||
        expression._operator == not_equal) {

      if (expression.operands.size() != 2) {
        throw std::runtime_error("SCIPSolver: Comparison operator requires exactly 2 operands");
      }

      // Build lhs and rhs expressions
      auto leftResult = buildExpression(model, expression.operands[0]);
      auto rightResult = buildExpression(model, expression.operands[1]);

      // Create lhs - rhs
      SCIP_EXPR* diffExpr;
      SCIP_EXPR* diffChildren[] = { leftResult, rightResult };
      double diffCoeffs[] = { 1.0, -1.0 };
      SCIPcreateExprSum(scip, &diffExpr, 2, diffChildren, diffCoeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &diffChildren[0]);
      SCIPreleaseExpr(scip, &diffChildren[1]);

      // Create auxiliary binary variable for comparison result
      size_t auxId = auxiliaryCounter++;
      std::string binaryName = "comp_aux_" + std::to_string(auxId);
      SCIP_VAR* binaryVar;
      SCIPcreateVarBasic(scip, &binaryVar, binaryName.c_str(), 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
      SCIPaddVar(scip, binaryVar);

      SCIP_EXPR* binaryExpr1;
      SCIPcreateExprVar(scip, &binaryExpr1, binaryVar, nullptr, nullptr);

      SCIP_EXPR* binaryExpr2;
      SCIPcreateExprVar(scip, &binaryExpr2, binaryVar, nullptr, nullptr);

      // Add epsilon-based constraints to enforce: binaryVar = 1 iff (lhs comp rhs)
      if (expression._operator == greater_or_equal) {
        // b ⟺ (lhs >= rhs)
        // True when: lhs - rhs >= -epsilon
        // False when: lhs - rhs <= -1.1 * epsilon

        // Constraint 1: b * (lhs - rhs + epsilon) >= 0
        SCIP_EXPR* expr1;
        double offset1 = epsilon;
        SCIPcreateExprSum(scip, &expr1, 1, &diffExpr, nullptr, offset1, nullptr, nullptr);
        SCIP_EXPR* product1;
        SCIP_EXPR* prod1Children[] = { binaryExpr1, expr1 };
        SCIPcreateExprProduct(scip, &product1, 2, prod1Children, 1.0, nullptr, nullptr);

        std::string cons1Name = "comp_true_" + std::to_string(auxId);
        SCIP_CONS* cons1;
        SCIPcreateConsBasicNonlinear(scip, &cons1, cons1Name.c_str(), product1, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons1);
        SCIPreleaseCons(scip, &cons1);
        SCIPreleaseExpr(scip, &product1);
        SCIPreleaseExpr(scip, &expr1);

        // Constraint 2: (1-b) * (-1.1*epsilon - (lhs - rhs)) >= 0
        SCIP_EXPR* oneMinusB;
        double negOne = -1.0;
        SCIPcreateExprSum(scip, &oneMinusB, 1, &binaryExpr2, &negOne, 1.0, nullptr, nullptr);

        SCIP_EXPR* expr2;
        SCIPcreateExprSum(scip, &expr2, 1, &diffExpr, &negOne, -1.1 * epsilon, nullptr, nullptr);

        SCIP_EXPR* product2;
        SCIP_EXPR* prod2Children[] = { oneMinusB, expr2 };
        SCIPcreateExprProduct(scip, &product2, 2, prod2Children, 1.0, nullptr, nullptr);

        std::string cons2Name = "comp_false_" + std::to_string(auxId);
        SCIP_CONS* cons2;
        SCIPcreateConsBasicNonlinear(scip, &cons2, cons2Name.c_str(), product2, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons2);
        SCIPreleaseCons(scip, &cons2);
        SCIPreleaseExpr(scip, &product2);
        SCIPreleaseExpr(scip, &expr2);
        SCIPreleaseExpr(scip, &oneMinusB);
      }
      else if (expression._operator == less_or_equal) {
        // b ⟺ (lhs <= rhs)
        // True when: lhs - rhs <= epsilon
        // False when: lhs - rhs >= 1.1 * epsilon

        // Constraint 1: b * (epsilon - (lhs - rhs)) >= 0
        double negOne = -1.0;
        SCIP_EXPR* expr1;
        double offset1 = epsilon;
        SCIPcreateExprSum(scip, &expr1, 1, &diffExpr, &negOne, offset1, nullptr, nullptr);
        SCIP_EXPR* product1;
        SCIP_EXPR* prod1Children[] = { binaryExpr1, expr1 };
        SCIPcreateExprProduct(scip, &product1, 2, prod1Children, 1.0, nullptr, nullptr);

        std::string cons1Name = "comp_true_" + std::to_string(auxId);
        SCIP_CONS* cons1;
        SCIPcreateConsBasicNonlinear(scip, &cons1, cons1Name.c_str(), product1, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons1);
        SCIPreleaseCons(scip, &cons1);
        SCIPreleaseExpr(scip, &product1);
        SCIPreleaseExpr(scip, &expr1);

        // Constraint 2: (1-b) * (lhs - rhs - 1.1*epsilon) >= 0
        SCIP_EXPR* oneMinusB;
        SCIPcreateExprSum(scip, &oneMinusB, 1, &binaryExpr2, &negOne, 1.0, nullptr, nullptr);

        SCIP_EXPR* expr2;
        SCIPcreateExprSum(scip, &expr2, 1, &diffExpr, nullptr, -1.1 * epsilon, nullptr, nullptr);

        SCIP_EXPR* product2;
        SCIP_EXPR* prod2Children[] = { oneMinusB, expr2 };
        SCIPcreateExprProduct(scip, &product2, 2, prod2Children, 1.0, nullptr, nullptr);

        std::string cons2Name = "comp_false_" + std::to_string(auxId);
        SCIP_CONS* cons2;
        SCIPcreateConsBasicNonlinear(scip, &cons2, cons2Name.c_str(), product2, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons2);
        SCIPreleaseCons(scip, &cons2);
        SCIPreleaseExpr(scip, &product2);
        SCIPreleaseExpr(scip, &expr2);
        SCIPreleaseExpr(scip, &oneMinusB);
      }
      else if (expression._operator == greater_than) {
        // b ⟺ (lhs > rhs)
        // True when: lhs - rhs >= 1.1 * epsilon (strictly larger)
        // False when: lhs - rhs <= epsilon

        // Constraint 1: b * (lhs - rhs - 1.1*epsilon) >= 0
        SCIP_EXPR* expr1;
        SCIPcreateExprSum(scip, &expr1, 1, &diffExpr, nullptr, -1.1 * epsilon, nullptr, nullptr);
        SCIP_EXPR* product1;
        SCIP_EXPR* prod1Children[] = { binaryExpr1, expr1 };
        SCIPcreateExprProduct(scip, &product1, 2, prod1Children, 1.0, nullptr, nullptr);

        std::string cons1Name = "comp_true_" + std::to_string(auxId);
        SCIP_CONS* cons1;
        SCIPcreateConsBasicNonlinear(scip, &cons1, cons1Name.c_str(), product1, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons1);
        SCIPreleaseCons(scip, &cons1);
        SCIPreleaseExpr(scip, &product1);
        SCIPreleaseExpr(scip, &expr1);

        // Constraint 2: (1-b) * (epsilon - (lhs - rhs)) >= 0
        SCIP_EXPR* oneMinusB;
        double negOne = -1.0;
        SCIPcreateExprSum(scip, &oneMinusB, 1, &binaryExpr2, &negOne, 1.0, nullptr, nullptr);

        SCIP_EXPR* expr2;
        double offset2 = epsilon;
        SCIPcreateExprSum(scip, &expr2, 1, &diffExpr, &negOne, offset2, nullptr, nullptr);

        SCIP_EXPR* product2;
        SCIP_EXPR* prod2Children[] = { oneMinusB, expr2 };
        SCIPcreateExprProduct(scip, &product2, 2, prod2Children, 1.0, nullptr, nullptr);

        std::string cons2Name = "comp_false_" + std::to_string(auxId);
        SCIP_CONS* cons2;
        SCIPcreateConsBasicNonlinear(scip, &cons2, cons2Name.c_str(), product2, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons2);
        SCIPreleaseCons(scip, &cons2);
        SCIPreleaseExpr(scip, &product2);
        SCIPreleaseExpr(scip, &expr2);
        SCIPreleaseExpr(scip, &oneMinusB);
      }
      else if (expression._operator == less_than) {
        // b ⟺ (lhs < rhs)
        // True when: lhs - rhs <= -1.1 * epsilon (strictly smaller)
        // False when: lhs - rhs >= -epsilon

        // Constraint 1: b * (-1.1*epsilon - (lhs - rhs)) >= 0
        double negOne = -1.0;
        SCIP_EXPR* expr1;
        SCIPcreateExprSum(scip, &expr1, 1, &diffExpr, &negOne, -1.1 * epsilon, nullptr, nullptr);
        SCIP_EXPR* product1;
        SCIP_EXPR* prod1Children[] = { binaryExpr1, expr1 };
        SCIPcreateExprProduct(scip, &product1, 2, prod1Children, 1.0, nullptr, nullptr);

        std::string cons1Name = "comp_true_" + std::to_string(auxId);
        SCIP_CONS* cons1;
        SCIPcreateConsBasicNonlinear(scip, &cons1, cons1Name.c_str(), product1, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons1);
        SCIPreleaseCons(scip, &cons1);
        SCIPreleaseExpr(scip, &product1);
        SCIPreleaseExpr(scip, &expr1);

        // Constraint 2: (1-b) * (lhs - rhs + epsilon) >= 0
        SCIP_EXPR* oneMinusB;
        SCIPcreateExprSum(scip, &oneMinusB, 1, &binaryExpr2, &negOne, 1.0, nullptr, nullptr);

        SCIP_EXPR* expr2;
        double offset2 = epsilon;
        SCIPcreateExprSum(scip, &expr2, 1, &diffExpr, nullptr, offset2, nullptr, nullptr);

        SCIP_EXPR* product2;
        SCIP_EXPR* prod2Children[] = { oneMinusB, expr2 };
        SCIPcreateExprProduct(scip, &product2, 2, prod2Children, 1.0, nullptr, nullptr);

        std::string cons2Name = "comp_false_" + std::to_string(auxId);
        SCIP_CONS* cons2;
        SCIPcreateConsBasicNonlinear(scip, &cons2, cons2Name.c_str(), product2, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons2);
        SCIPreleaseCons(scip, &cons2);
        SCIPreleaseExpr(scip, &product2);
        SCIPreleaseExpr(scip, &expr2);
        SCIPreleaseExpr(scip, &oneMinusB);
      }
      else if (expression._operator == equal) {
        // b ⟺ (lhs == rhs)
        // True when: |lhs - rhs| <= epsilon
        // False when: |lhs - rhs| >= 1.1 * epsilon

        // Constraint 1: b * (epsilon - (lhs - rhs)) >= 0
        SCIP_EXPR* expr1;
        double negOne = -1.0;
        SCIPcreateExprSum(scip, &expr1, 1, &diffExpr, &negOne, epsilon, nullptr, nullptr);
        SCIP_EXPR* product1;
        SCIP_EXPR* prod1Children[] = { binaryExpr1, expr1 };
        SCIPcreateExprProduct(scip, &product1, 2, prod1Children, 1.0, nullptr, nullptr);

        std::string cons1Name = "comp_true_lower_" + std::to_string(auxId);
        SCIP_CONS* cons1;
        SCIPcreateConsBasicNonlinear(scip, &cons1, cons1Name.c_str(), product1, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons1);
        SCIPreleaseCons(scip, &cons1);
        SCIPreleaseExpr(scip, &product1);
        SCIPreleaseExpr(scip, &expr1);

        // Constraint 2: b * (epsilon + (lhs - rhs)) >= 0
        SCIP_EXPR* binaryExpr3;
        SCIPcreateExprVar(scip, &binaryExpr3, binaryVar, nullptr, nullptr);

        SCIP_EXPR* expr2;
        SCIPcreateExprSum(scip, &expr2, 1, &diffExpr, nullptr, epsilon, nullptr, nullptr);
        SCIP_EXPR* product2;
        SCIP_EXPR* prod2Children[] = { binaryExpr3, expr2 };
        SCIPcreateExprProduct(scip, &product2, 2, prod2Children, 1.0, nullptr, nullptr);

        std::string cons2Name = "comp_true_upper_" + std::to_string(auxId);
        SCIP_CONS* cons2;
        SCIPcreateConsBasicNonlinear(scip, &cons2, cons2Name.c_str(), product2, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons2);
        SCIPreleaseCons(scip, &cons2);
        SCIPreleaseExpr(scip, &product2);
        SCIPreleaseExpr(scip, &expr2);
        SCIPreleaseExpr(scip, &binaryExpr3);

        // Constraint 3: (1-b) * (|lhs - rhs| - 1.1*epsilon) >= 0
        SCIP_EXPR* oneMinusB;
        SCIPcreateExprSum(scip, &oneMinusB, 1, &binaryExpr2, &negOne, 1.0, nullptr, nullptr);

        SCIP_EXPR* absExpr;
        SCIPcreateExprAbs(scip, &absExpr, diffExpr, nullptr, nullptr);

        SCIP_EXPR* expr3;
        double offset3 = -1.1 * epsilon;
        SCIPcreateExprSum(scip, &expr3, 1, &absExpr, nullptr, offset3, nullptr, nullptr);

        SCIP_EXPR* product3;
        SCIP_EXPR* prod3Children[] = { oneMinusB, expr3 };
        SCIPcreateExprProduct(scip, &product3, 2, prod3Children, 1.0, nullptr, nullptr);

        std::string cons3Name = "comp_false_" + std::to_string(auxId);
        SCIP_CONS* cons3;
        SCIPcreateConsBasicNonlinear(scip, &cons3, cons3Name.c_str(), product3, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons3);
        SCIPreleaseCons(scip, &cons3);
        SCIPreleaseExpr(scip, &product3);
        SCIPreleaseExpr(scip, &expr3);
        SCIPreleaseExpr(scip, &absExpr);
        SCIPreleaseExpr(scip, &oneMinusB);
      }
      else if (expression._operator == not_equal) {
        // b ⟺ (lhs != rhs)
        // True when: |lhs - rhs| >= 1.1 * epsilon
        // False when: |lhs - rhs| <= epsilon

        // Constraint 1: b * (|lhs - rhs| - 1.1*epsilon) >= 0
        SCIP_EXPR* absExpr1;
        SCIPcreateExprAbs(scip, &absExpr1, diffExpr, nullptr, nullptr);

        SCIP_EXPR* expr1;
        double offset1 = -1.1 * epsilon;
        SCIPcreateExprSum(scip, &expr1, 1, &absExpr1, nullptr, offset1, nullptr, nullptr);
        SCIP_EXPR* product1;
        SCIP_EXPR* prod1Children[] = { binaryExpr1, expr1 };
        SCIPcreateExprProduct(scip, &product1, 2, prod1Children, 1.0, nullptr, nullptr);

        std::string cons1Name = "comp_true_" + std::to_string(auxId);
        SCIP_CONS* cons1;
        SCIPcreateConsBasicNonlinear(scip, &cons1, cons1Name.c_str(), product1, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons1);
        SCIPreleaseCons(scip, &cons1);
        SCIPreleaseExpr(scip, &product1);
        SCIPreleaseExpr(scip, &expr1);
        SCIPreleaseExpr(scip, &absExpr1);

        // Constraint 2: (1-b) * (epsilon - |lhs - rhs|) >= 0
        SCIP_EXPR* oneMinusB;
        double negOne = -1.0;
        SCIPcreateExprSum(scip, &oneMinusB, 1, &binaryExpr2, &negOne, 1.0, nullptr, nullptr);

        SCIP_EXPR* absExpr2;
        SCIPcreateExprAbs(scip, &absExpr2, diffExpr, nullptr, nullptr);

        SCIP_EXPR* expr2;
        SCIPcreateExprSum(scip, &expr2, 1, &absExpr2, &negOne, epsilon, nullptr, nullptr);

        SCIP_EXPR* product2;
        SCIP_EXPR* prod2Children[] = { oneMinusB, expr2 };
        SCIPcreateExprProduct(scip, &product2, 2, prod2Children, 1.0, nullptr, nullptr);

        std::string cons2Name = "comp_false_" + std::to_string(auxId);
        SCIP_CONS* cons2;
        SCIPcreateConsBasicNonlinear(scip, &cons2, cons2Name.c_str(), product2, 0.0, SCIPinfinity(scip));
        SCIPaddCons(scip, cons2);
        SCIPreleaseCons(scip, &cons2);
        SCIPreleaseExpr(scip, &product2);
        SCIPreleaseExpr(scip, &expr2);
        SCIPreleaseExpr(scip, &absExpr2);
        SCIPreleaseExpr(scip, &oneMinusB);
      }

      SCIPreleaseExpr(scip, &binaryExpr1);
      SCIPreleaseExpr(scip, &binaryExpr2);
      SCIPreleaseExpr(scip, &diffExpr);

      // Return expression for the binary variable
      SCIP_EXPR* resultExpr;
      SCIPcreateExprVar(scip, &resultExpr, binaryVar, nullptr, nullptr);
      SCIPreleaseVar(scip, &binaryVar);
      return resultExpr;
    }

    throw std::runtime_error("SCIPSolver: Unsupported expression operator");
  }

  throw std::runtime_error("SCIPSolver: Unsupported operand type");
}

void SCIPSolver::addDeducedConstraints(const Model& model) {
  // Process deducedFrom constraints for regular variables
  for (const auto& variable : model.getVariables()) {
    if (variable.deducedFrom) {
      auto it = variableMap.find(&variable);
      if (it == variableMap.end()) {
        throw std::runtime_error("SCIPSolver: Variable " + variable.name + " not found in variableMap");
      }
      SCIP_VAR* scipVar = it->second;

      auto expressionResult = buildExpression(model, *variable.deducedFrom);

      // Create var - expression == 0 constraint
      SCIP_EXPR* varExpr;
      SCIPcreateExprVar(scip, &varExpr, scipVar, nullptr, nullptr);

      SCIP_EXPR* diffChildren[] = { varExpr, expressionResult };
      double coeffs[] = { 1.0, -1.0 };
      SCIP_EXPR* diffExpr;
      SCIPcreateExprSum(scip, &diffExpr, 2, diffChildren, coeffs, 0.0, nullptr, nullptr);

      std::string consName = "deduced_" + variable.name;
      SCIP_CONS* cons;
      SCIPcreateConsBasicNonlinear(scip, &cons, consName.c_str(), diffExpr, 0.0, 0.0);
      SCIPaddCons(scip, cons);
      SCIPreleaseCons(scip, &cons);
      SCIPreleaseExpr(scip, &diffExpr);
      SCIPreleaseExpr(scip, &varExpr);
      SCIPreleaseExpr(scip, &expressionResult);
    }
  }

  // Process deducedFrom constraints for indexed variables
  for (const auto& indexedVariables : model.getIndexedVariables()) {
    for (const auto& indexedVariable : indexedVariables) {
      if (indexedVariable.deducedFrom) {
        auto it = variableMap.find(&indexedVariable);
        if (it == variableMap.end()) {
          throw std::runtime_error("SCIPSolver: Indexed variable " + indexedVariable.name + " not found in variableMap");
        }
        SCIP_VAR* scipVar = it->second;

        auto expressionResult = buildExpression(model, *indexedVariable.deducedFrom);

        // Create var - expression == 0 constraint
        SCIP_EXPR* varExpr;
        SCIPcreateExprVar(scip, &varExpr, scipVar, nullptr, nullptr);

        SCIP_EXPR* diffChildren[] = { varExpr, expressionResult };
        double coeffs[] = { 1.0, -1.0 };
        SCIP_EXPR* diffExpr;
        SCIPcreateExprSum(scip, &diffExpr, 2, diffChildren, coeffs, 0.0, nullptr, nullptr);

        std::string consName = "deduced_" + indexedVariable.name;
        SCIP_CONS* cons;
        SCIPcreateConsBasicNonlinear(scip, &cons, consName.c_str(), diffExpr, 0.0, 0.0);
        SCIPaddCons(scip, cons);
        SCIPreleaseCons(scip, &cons);
        SCIPreleaseExpr(scip, &diffExpr);
        SCIPreleaseExpr(scip, &varExpr);
        SCIPreleaseExpr(scip, &expressionResult);
      }
    }
  }
}

void SCIPSolver::addObjective(const Model& model) {
  // Set objective sense
  if (model.getObjectiveSense() == Model::ObjectiveSense::MINIMIZE) {
    SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE);
  }
  else if (model.getObjectiveSense() == Model::ObjectiveSense::MAXIMIZE) {
    SCIPsetObjsense(scip, SCIP_OBJSENSE_MAXIMIZE);
  }
  else {
    // FEASIBLE - no objective
    return;
  }

  const auto& objective = model.getObjective();

  // Build objective expression - wrap in operand
  Operand objOperand = objective;
  auto objExpr = buildExpression(model, objOperand);

  // Create auxiliary variable for objective with coefficient 1.0
  SCIP_VAR* objVar;
  SCIPcreateVarBasic(scip, &objVar, "obj", -SCIPinfinity(scip), SCIPinfinity(scip), 1.0, SCIP_VARTYPE_CONTINUOUS);
  SCIPaddVar(scip, objVar);

  // Create constraint: objVar == objective_expression
  SCIP_EXPR* objVarExpr;
  SCIPcreateExprVar(scip, &objVarExpr, objVar, nullptr, nullptr);

  SCIP_EXPR* diffExpr;
  SCIP_EXPR* children[] = { objVarExpr, objExpr };
  double coeffs[] = { 1.0, -1.0 };
  SCIPcreateExprSum(scip, &diffExpr, 2, children, coeffs, 0.0, nullptr, nullptr);

  SCIP_CONS* objCons;
  SCIPcreateConsBasicNonlinear(scip, &objCons, "obj_constraint", diffExpr, 0.0, 0.0);
  SCIPaddCons(scip, objCons);

  SCIPreleaseCons(scip, &objCons);
  SCIPreleaseExpr(scip, &diffExpr);
  SCIPreleaseExpr(scip, &objVarExpr);
  SCIPreleaseExpr(scip, &objExpr);
  SCIPreleaseVar(scip, &objVar);
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
        throw std::runtime_error("SCIPSolver: malformed constraint" );
      }

      // Build expression for lhs - rhs
      auto leftExpr = buildExpression(model, constraint.operands[0]);
      auto rightExpr = buildExpression(model, constraint.operands[1]);

      // Create lhs - rhs expression
      SCIP_EXPR* diffExpr;
      SCIP_EXPR* children[] = { leftExpr, rightExpr };
      double coeffs[] = { 1.0, -1.0 };
      SCIPcreateExprSum(scip, &diffExpr, 2, children, coeffs, 0.0, nullptr, nullptr);
      SCIPreleaseExpr(scip, &children[0]);
      SCIPreleaseExpr(scip, &children[1]);

      // Determine bounds: lhs - rhs {<=, >=, ==} 0
      double lhs = -SCIPinfinity(scip);
      double rhs = SCIPinfinity(scip);

      std::string consName = "cons_" + std::to_string(i);

      if (constraint._operator == less_or_equal) {
        // expr <= 0  =>  expr <= epsilon
        rhs = epsilon;
      }
      else if (constraint._operator == less_than) {
        // expr < 0  =>  expr <= -1.1*epsilon
        rhs = -1.1 * epsilon;
      }
      else if (constraint._operator == greater_or_equal) {
        // expr >= 0  =>  expr >= -epsilon
        lhs = -epsilon;
      }
      else if (constraint._operator == greater_than) {
        // expr > 0  =>  expr >= 1.1*epsilon
        lhs = 1.1 * epsilon;
      }
      else if (constraint._operator == equal) {
        // expr == 0  =>  -epsilon <= expr <= epsilon
        lhs = -epsilon;
        rhs = epsilon;
      }
      else if (constraint._operator == not_equal) {
        // x != y  =>  |x - y| >= 1.1*epsilon
        // Use SCIP's built-in absolute value expression
        SCIP_EXPR* absExpr;
        SCIPcreateExprAbs(scip, &absExpr, diffExpr, nullptr, nullptr);

        SCIP_CONS* absGeCons;
        SCIPcreateConsBasicNonlinear(scip, &absGeCons, consName.c_str(), absExpr,
                      1.1 * epsilon, SCIPinfinity(scip));
        SCIPaddCons(scip, absGeCons);
        SCIPreleaseCons(scip, &absGeCons);
        SCIPreleaseExpr(scip, &absExpr);
        SCIPreleaseExpr(scip, &diffExpr);

        continue;
      }

      // Create nonlinear constraint
      SCIP_CONS* cons;
      SCIPcreateConsBasicNonlinear(scip, &cons, consName.c_str(), diffExpr, lhs, rhs);
      SCIPaddCons(scip, cons);
      SCIPreleaseCons(scip, &cons);
      SCIPreleaseExpr(scip, &diffExpr);
    }
    else {
      // Handle other boolean constraints (logical_or, logical_and, etc.)
      auto constraintExpr = buildExpression(model, constraint);

      // Enforce constraintExpr >= 1 - epsilon (i.e., must be true)
      std::string consName = "cons_" + std::to_string(i);
      SCIP_CONS* cons;
      SCIPcreateConsBasicNonlinear(scip, &cons, consName.c_str(), constraintExpr, 1.0 - epsilon, SCIPinfinity(scip));
      SCIPaddCons(scip, cons);
      SCIPreleaseCons(scip, &cons);
      SCIPreleaseExpr(scip, &constraintExpr);
    }
  }
}

SCIP_EXPR* SCIPSolver::addIndexingConstraints(const std::string& name, const std::vector<SCIP_VAR*>& arrayVars, SCIP_VAR* indexVar, SCIP_VAR* resultVar, int indexOffset) {
  size_t n = arrayVars.size();

  // Create binary variables b[i] for each position i
  std::vector<SCIP_VAR*> binaries(n);
  for (size_t i = 0; i < n; i++) {
    std::string binName = name + "_b[" + std::to_string(i) + "]";
    SCIP_VAR* binVar;
    SCIPcreateVarBasic(scip, &binVar, binName.c_str(), 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
    SCIPaddVar(scip, binVar);
    binaries[i] = binVar;
  }

  // Constraint: sum(b[i]) = 1 (exactly one position selected)
  SCIP_CONS* sumCons;
  std::vector<double> ones(n, 1.0);
  std::string sumName = name + "_sum";
  SCIPcreateConsBasicLinear(scip, &sumCons, sumName.c_str(), n, binaries.data(), ones.data(), 1.0, 1.0);
  SCIPaddCons(scip, sumCons);
  SCIPreleaseCons(scip, &sumCons);

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
  SCIPcreateConsBasicLinear(scip, &indexCons, indexName.c_str(), indexVars.size(), indexVars.data(), indexCoeffs.data(), -epsilon, epsilon);
  SCIPaddCons(scip, indexCons);
  SCIPreleaseCons(scip, &indexCons);

  // Constraint: result = sum(b[i] * array[i])
  // This is algebraic (no big-M), consistent with if-then-else implementation
  std::vector<SCIP_EXPR*> productExprs;

  for (size_t i = 0; i < n; i++) {
    // Create b[i] * array[i]
    SCIP_EXPR* binExpr;
    SCIPcreateExprVar(scip, &binExpr, binaries[i], nullptr, nullptr);

    SCIP_EXPR* arrayExpr;
    SCIPcreateExprVar(scip, &arrayExpr, arrayVars[i], nullptr, nullptr);

    SCIP_EXPR* productExpr;
    SCIP_EXPR* factors[] = { binExpr, arrayExpr };
    SCIPcreateExprProduct(scip, &productExpr, 2, factors, 1.0, nullptr, nullptr);

    productExprs.push_back(productExpr);

    SCIPreleaseExpr(scip, &binExpr);
    SCIPreleaseExpr(scip, &arrayExpr);
  }

  // Create sum of products
  SCIP_EXPR* sumExpr;
  std::vector<double> coeffs(productExprs.size(), 1.0);
  SCIPcreateExprSum(scip, &sumExpr, productExprs.size(), productExprs.data(),
      coeffs.data(), 0.0, nullptr, nullptr);

  // Create result variable expression
  SCIP_EXPR* resultExpr_tmp;
  SCIPcreateExprVar(scip, &resultExpr_tmp, resultVar, nullptr, nullptr);

  // Constraint: result - sum(b[i] * array[i]) = 0
  SCIP_EXPR* diffExpr;
  SCIP_EXPR* diffChildren[] = { resultExpr_tmp, sumExpr };
  double diffCoeffs[] = { 1.0, -1.0 };
  SCIPcreateExprSum(scip, &diffExpr, 2, diffChildren, diffCoeffs, 0.0, nullptr, nullptr);

  SCIP_CONS* resultCons;
  std::string resultConsName = name + "_result";
  SCIPcreateConsBasicNonlinear(scip, &resultCons, resultConsName.c_str(), diffExpr, 0.0, 0.0);
  SCIPaddCons(scip, resultCons);
  SCIPreleaseCons(scip, &resultCons);

  // Release expressions
  SCIPreleaseExpr(scip, &diffExpr);
  SCIPreleaseExpr(scip, &resultExpr_tmp);
  SCIPreleaseExpr(scip, &sumExpr);
  for (auto scipExpr : productExprs) {
    SCIPreleaseExpr(scip, &scipExpr);
  }

  // Release binary variables
  for (size_t i = 0; i < n; i++) {
    SCIPreleaseVar(scip, &binaries[i]);
  }

  // Return expression for result variable
  SCIP_EXPR* resultExpr;
  SCIPcreateExprVar(scip, &resultExpr, resultVar, nullptr, nullptr);
  return resultExpr;
}

std::expected<Solution, std::string> SCIPSolver::solve(const Model& model) {
  return solve(model, std::numeric_limits<double>::infinity());
}

std::expected<Solution, std::string> SCIPSolver::solve(const Model& model, double timeLimit) {
  // Set or unset time limit
  if (std::isfinite(timeLimit)) {
    // Set finite time limit
    SCIPsetRealParam(scip, "limits/time", timeLimit);
  }
  else {
    // Unset time limit by setting to SCIP's infinity
    SCIPsetRealParam(scip, "limits/time", SCIPinfinity(scip));
  }

  // Solve the problem
  SCIP_RETCODE retcode = SCIPsolve(scip);
  if (retcode != SCIP_OKAY) {
    return std::unexpected("SCIP solve failed");
  }

  // Check SCIP status
  SCIP_STATUS scipStatus = SCIPgetStatus(scip);

  // Get the best solution
  SCIP_SOL* sol = SCIPgetBestSol(scip);

  // Create CP solution object
  Solution solution(model);

  // Map SCIP status to Solution::Status
  switch (scipStatus) {
    case SCIP_STATUS_OPTIMAL:
      solution.setStatus(Solution::Status::OPTIMAL);
      break;
    case SCIP_STATUS_BESTSOLLIMIT:
    case SCIP_STATUS_GAPLIMIT:
    case SCIP_STATUS_SOLLIMIT:
    case SCIP_STATUS_STALLNODELIMIT:
    case SCIP_STATUS_TIMELIMIT:
    case SCIP_STATUS_MEMLIMIT:
    case SCIP_STATUS_NODELIMIT:
    case SCIP_STATUS_TOTALNODELIMIT:
    case SCIP_STATUS_USERINTERRUPT:
      // Stopped before proving optimality - solution is feasible but not proven optimal
      if (sol) {
        solution.setStatus(Solution::Status::FEASIBLE);
      } else {
        solution.setStatus(Solution::Status::UNKNOWN);
      }
      break;
    case SCIP_STATUS_INFEASIBLE:
      solution.setStatus(Solution::Status::INFEASIBLE);
      return std::unexpected("Problem is infeasible");
    case SCIP_STATUS_UNBOUNDED:
      solution.setStatus(Solution::Status::UNBOUNDED);
      return std::unexpected("Problem is unbounded");
    default:
      solution.setStatus(Solution::Status::UNKNOWN);
  }

  // Check if we have a solution to extract
  if (!sol) {
    return std::unexpected("No solution found");
  }

  // Extract variable values from SCIP solution with precision rounding for CP solution
  for (const auto& [variable, scipVar] : variableMap) {
    double value = SCIPgetSolVal(scip, sol, scipVar);

    // Round to precision decimal places for CP solution generation
    double factor = std::pow(10.0, precision);
    value = std::round(value * factor) / factor;

    solution.setVariableValue(*variable, value);
  }

  return solution;
}

// Collection operation helpers

SCIP_EXPR* SCIPSolver::resolveCollectionOperation(
    const Model& model,
    const Expression& expression,
    const std::string& opName
) {
  // Extract key variable from collection(keyVar)
  const Expression& collectionExpr = std::get<Expression>(expression.operands[1]);

  if (collectionExpr.operands.size() != 1) {
    throw std::runtime_error("SCIPSolver: collection() must have exactly 1 argument");
  }

  const Operand& keyOperand = collectionExpr.operands[0];

  // Handle constant key - return constant result
  auto constantKeyOpt = extractConstant(keyOperand);
  if (constantKeyOpt.has_value()) {
    double constantKey = constantKeyOpt.value();
    auto collectionResult = model.getCollection(constantKey);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(constantKey) +
                               " not found in model");
    }

    const std::vector<double>& collection = collectionResult.value();
    double result;

    if (opName == "count") {
      result = (double)collection.size();
    }
    else if (opName == "sum") {
      result = std::accumulate(collection.begin(), collection.end(), 0.0);
    }
    else if (opName == "avg") {
      if (collection.empty()) {
        throw std::runtime_error("SCIPSolver: avg() is undefined for empty collection");
      }
      result = std::accumulate(collection.begin(), collection.end(), 0.0) / collection.size();
    }
    else if (opName == "max") {
      if (collection.empty()) {
        throw std::runtime_error("SCIPSolver: max() is undefined for empty collection");
      }
      result = *std::max_element(collection.begin(), collection.end());
    }
    else if (opName == "min") {
      if (collection.empty()) {
        throw std::runtime_error("SCIPSolver: min() is undefined for empty collection");
      }
      result = *std::min_element(collection.begin(), collection.end());
    }

    // Return constant value expression
    SCIP_EXPR* resultExpr;
    SCIPcreateExprValue(scip, &resultExpr, result, nullptr, nullptr);
    return resultExpr;
  }

  // Unwrap Expression(none, {operand}) to get the actual operand
  const Operand* actualKeyOperand = &keyOperand;
  if (std::holds_alternative<Expression>(keyOperand)) {
    const Expression& keyExpr = std::get<Expression>(keyOperand);
    if (keyExpr._operator == Expression::Operator::none && keyExpr.operands.size() == 1) {
      actualKeyOperand = &keyExpr.operands[0];
    }
  }

  SCIP_VAR* scipKeyVar = nullptr;
  bool releaseScipKeyVar = false;  // Track if we need to release the auxiliary variable

  // Handle variable key
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
    const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand).get();
    auto it = variableMap.find(&keyVar);
    if (it == variableMap.end()) {
      throw std::runtime_error("SCIPSolver: Collection key variable not found");
    }
    scipKeyVar = it->second;
  }
  // Handle IndexedVariable key
  else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
    const IndexedVariable& idxVar = std::get<IndexedVariable>(*actualKeyOperand);

    // Build expression for the indexed variable
    SCIP_EXPR* keyScipExpr = buildExpression(model, *actualKeyOperand);

    // Get bounds from the IndexedVariables container's variables
    const IndexedVariables& container = idxVar.container.get();
    double minBound = std::numeric_limits<double>::max();
    double maxBound = std::numeric_limits<double>::lowest();

    for (const auto& var : container) {
      auto it = variableMap.find(&var);
      if (it != variableMap.end()) {
        double lb = SCIPvarGetLbGlobal(it->second);
        double ub = SCIPvarGetUbGlobal(it->second);
        minBound = std::min(minBound, lb);
        maxBound = std::max(maxBound, ub);
      }
    }

    // Create auxiliary variable for the collection key
    size_t auxId = auxiliaryCounter++;
    std::string auxName = "aux_collection_key_" + std::to_string(auxId);
    SCIP_VAR* auxKeyVar;
    SCIPcreateVarBasic(scip, &auxKeyVar, auxName.c_str(), minBound, maxBound,
                       0.0, SCIP_VARTYPE_INTEGER);
    SCIPaddVar(scip, auxKeyVar);

    // Add constraint: auxKeyVar == keyScipExpr
    SCIP_EXPR* auxExpr;
    SCIPcreateExprVar(scip, &auxExpr, auxKeyVar, nullptr, nullptr);

    SCIP_EXPR* diffExprs[] = { auxExpr, keyScipExpr };
    double coeffs[] = { 1.0, -1.0 };
    SCIP_EXPR* diffExpr;
    SCIPcreateExprSum(scip, &diffExpr, 2, diffExprs, coeffs, 0.0, nullptr, nullptr);

    SCIP_CONS* auxCons;
    std::string consName = "aux_collection_key_cons_" + std::to_string(auxId);
    SCIPcreateConsBasicNonlinear(scip, &auxCons, consName.c_str(), diffExpr, 0.0, 0.0);
    SCIPaddCons(scip, auxCons);
    SCIPreleaseCons(scip, &auxCons);
    SCIPreleaseExpr(scip, &diffExpr);
    SCIPreleaseExpr(scip, &auxExpr);
    SCIPreleaseExpr(scip, &keyScipExpr);

    scipKeyVar = auxKeyVar;
    releaseScipKeyVar = true;  // Mark for cleanup
  }
  else {
    throw std::runtime_error(
      "SCIPSolver: collection() key must be a variable or constant"
    );
  }

  // Get key bounds
  double lb = SCIPvarGetLbGlobal(scipKeyVar);
  double ub = SCIPvarGetUbGlobal(scipKeyVar);

  // Pre-compute ONLY the requested operation (lazy evaluation)
  int numKeys = (int)std::floor(ub) - (int)std::ceil(lb) + 1;
  std::vector<double> results(numKeys);

  for (int i = 0; i < numKeys; i++) {
    int key = (int)std::ceil(lb) + i;

    auto collectionResult = model.getCollection((double)key);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                               " not found in model");
    }

    const std::vector<double>& collection = collectionResult.value();

    // Compute only the requested operation
    if (opName == "count") {
      results[i] = (double)collection.size();
    }
    else if (opName == "sum") {
      results[i] = std::accumulate(collection.begin(), collection.end(), 0.0);
    }
    else if (opName == "avg") {
      if (collection.empty()) {
        throw std::runtime_error("SCIPSolver: avg() is undefined for empty collection at key " +
                                 std::to_string(key));
      }
      results[i] = std::accumulate(collection.begin(), collection.end(), 0.0) / collection.size();
    }
    else if (opName == "max") {
      if (collection.empty()) {
        throw std::runtime_error("SCIPSolver: max() is undefined for empty collection at key " +
                                 std::to_string(key));
      }
      results[i] = *std::max_element(collection.begin(), collection.end());
    }
    else if (opName == "min") {
      if (collection.empty()) {
        throw std::runtime_error("SCIPSolver: min() is undefined for empty collection at key " +
                                 std::to_string(key));
      }
      results[i] = *std::min_element(collection.begin(), collection.end());
    }
    else {
      throw std::runtime_error("SCIPSolver: Unknown aggregate operation: " + opName);
    }
  }

  // Use element constraint to select result
  SCIP_EXPR* result = buildElementConstraint(results, scipKeyVar, lb);

  // Release auxiliary variable if created
  if (releaseScipKeyVar) {
    SCIPreleaseVar(scip, &scipKeyVar);
  }

  return result;
}

SCIP_EXPR* SCIPSolver::buildElementConstraint(
    const std::vector<double>& values,
    SCIP_VAR* indexVar,
    double indexOffset
) {
  // Create SCIP variables for the pre-computed values (fixed values)
  std::vector<SCIP_VAR*> valueVars;

  for (size_t i = 0; i < values.size(); i++) {
    SCIP_VAR* var;
    std::string name = "param_" + std::to_string(auxiliaryCounter++);

    // Create variable with fixed bounds (value[i] == value[i])
    SCIPcreateVarBasic(
      scip,
      &var,
      name.c_str(),
      values[i],      // lower bound = value
      values[i],      // upper bound = value
      0.0,            // obj coeff
      SCIP_VARTYPE_CONTINUOUS
    );

    SCIPaddVar(scip, var);
    valueVars.push_back(var);
  }

  // Create result variable
  SCIP_VAR* resultVar;
  std::string resultName = "element_result_" + std::to_string(auxiliaryCounter++);

  // Bounds: min and max of values array
  double minVal = *std::min_element(values.begin(), values.end());
  double maxVal = *std::max_element(values.begin(), values.end());

  SCIPcreateVarBasic(
    scip,
    &resultVar,
    resultName.c_str(),
    minVal,
    maxVal,
    0.0,
    SCIP_VARTYPE_CONTINUOUS
  );

  SCIPaddVar(scip, resultVar);

  // Use existing addIndexingConstraints helper (returns expression for resultVar)
  SCIP_EXPR* resultExpr = addIndexingConstraints(
    resultName,
    valueVars,
    indexVar,
    resultVar,
    (int)indexOffset  // Offset to convert key to 0-based index
  );

  // Release temporary variables (SCIP holds references)
  for (auto var : valueVars) {
    SCIPreleaseVar(scip, &var);
  }
  SCIPreleaseVar(scip, &resultVar);

  return resultExpr;
}

SCIP_EXPR* SCIPSolver::resolveCollectionMembership(
    const Model& model,
    const Expression& expression,
    const std::string& opName
) {
  // element_of(value, collection(key)) or not_element_of(value, collection(key))
  // operands[0] = custom operator index
  // operands[1] = value (constant or variable)
  // operands[2] = collection expression

  if (expression.operands.size() != 3) {
    throw std::runtime_error("SCIPSolver: " + opName + " requires exactly 2 arguments");
  }

  const Operand& valueOperand = expression.operands[1];
  const Expression& collectionExpr = std::get<Expression>(expression.operands[2]);

  // Extract key from collection(key)
  if (collectionExpr.operands.size() != 1) {
    throw std::runtime_error("SCIPSolver: collection() must have exactly 1 argument");
  }

  const Operand& keyOperand = collectionExpr.operands[0];

  // Handle constant key
  auto constantKeyOpt = extractConstant(keyOperand);
  if (constantKeyOpt.has_value()) {
    double constantKey = constantKeyOpt.value();
    auto collectionResult = model.getCollection(constantKey);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(constantKey) +
                               " not found in model");
    }

    const std::vector<double>& collection = collectionResult.value();

    // With constant key, just check membership for the value
    auto constantValueOpt = extractConstant(valueOperand);
    if (constantValueOpt.has_value()) {
      double constantValue = constantValueOpt.value();
      bool found = std::find(collection.begin(), collection.end(), constantValue) != collection.end();
      double result = (opName == "element_of") ? (found ? 1.0 : 0.0) : (found ? 0.0 : 1.0);

      SCIP_EXPR* resultExpr;
      SCIPcreateExprValue(scip, &resultExpr, result, nullptr, nullptr);
      return resultExpr;
    } else {
      throw std::runtime_error(
        "SCIPSolver: " + opName + " with constant key requires constant value"
      );
    }
  }

  // Unwrap Expression(none, {operand}) to get the actual operand
  const Operand* actualKeyOperand = &keyOperand;
  if (std::holds_alternative<Expression>(keyOperand)) {
    const Expression& keyExpr = std::get<Expression>(keyOperand);
    if (keyExpr._operator == Expression::Operator::none && keyExpr.operands.size() == 1) {
      actualKeyOperand = &keyExpr.operands[0];
    }
  }

  SCIP_VAR* scipKeyVar = nullptr;
  bool releaseScipKeyVar = false;  // Track if we need to release the auxiliary variable

  // Handle variable key
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
    const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand).get();
    auto it = variableMap.find(&keyVar);
    if (it == variableMap.end()) {
      throw std::runtime_error("SCIPSolver: Collection key variable not found");
    }
    scipKeyVar = it->second;
  }
  // Handle IndexedVariable key
  else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
    const IndexedVariable& idxVar = std::get<IndexedVariable>(*actualKeyOperand);

    // Build expression for the indexed variable
    SCIP_EXPR* keyScipExpr = buildExpression(model, *actualKeyOperand);

    // Get bounds from the IndexedVariables container's variables
    const IndexedVariables& container = idxVar.container.get();
    double minBound = std::numeric_limits<double>::max();
    double maxBound = std::numeric_limits<double>::lowest();

    for (const auto& var : container) {
      auto it = variableMap.find(&var);
      if (it != variableMap.end()) {
        double lb = SCIPvarGetLbGlobal(it->second);
        double ub = SCIPvarGetUbGlobal(it->second);
        minBound = std::min(minBound, lb);
        maxBound = std::max(maxBound, ub);
      }
    }

    // Create auxiliary variable for the collection key
    size_t auxId = auxiliaryCounter++;
    std::string auxName = "aux_collection_key_" + std::to_string(auxId);
    SCIP_VAR* auxKeyVar;
    SCIPcreateVarBasic(scip, &auxKeyVar, auxName.c_str(), minBound, maxBound,
                       0.0, SCIP_VARTYPE_INTEGER);
    SCIPaddVar(scip, auxKeyVar);

    // Add constraint: auxKeyVar == keyScipExpr
    SCIP_EXPR* auxExpr;
    SCIPcreateExprVar(scip, &auxExpr, auxKeyVar, nullptr, nullptr);

    SCIP_EXPR* diffExprs[] = { auxExpr, keyScipExpr };
    double coeffs[] = { 1.0, -1.0 };
    SCIP_EXPR* diffExpr;
    SCIPcreateExprSum(scip, &diffExpr, 2, diffExprs, coeffs, 0.0, nullptr, nullptr);

    SCIP_CONS* auxCons;
    std::string consName = "aux_collection_key_cons_" + std::to_string(auxId);
    SCIPcreateConsBasicNonlinear(scip, &auxCons, consName.c_str(), diffExpr, 0.0, 0.0);
    SCIPaddCons(scip, auxCons);
    SCIPreleaseCons(scip, &auxCons);
    SCIPreleaseExpr(scip, &diffExpr);
    SCIPreleaseExpr(scip, &auxExpr);
    SCIPreleaseExpr(scip, &keyScipExpr);

    scipKeyVar = auxKeyVar;
    releaseScipKeyVar = true;  // Mark for cleanup
  }
  else {
    throw std::runtime_error(
      "SCIPSolver: collection() key must be a variable or constant"
    );
  }

  // Get key bounds
  double keyLb = SCIPvarGetLbGlobal(scipKeyVar);
  double keyUb = SCIPvarGetUbGlobal(scipKeyVar);
  int numKeys = (int)std::floor(keyUb) - (int)std::ceil(keyLb) + 1;

  // Case A: Value is a constant
  if (std::holds_alternative<double>(valueOperand)) {
    double constantValue = std::get<double>(valueOperand);

    // Pre-compute membership for this constant value
    std::vector<double> membership(numKeys);

    for (int i = 0; i < numKeys; i++) {
      int key = (int)std::ceil(keyLb) + i;
      auto collectionResult = model.getCollection((double)key);
      if (!collectionResult) {
        throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                                 " not found in model");
      }

      const std::vector<double>& collection = collectionResult.value();
      bool found = std::find(collection.begin(), collection.end(), constantValue) != collection.end();

      membership[i] = (opName == "element_of") ? (found ? 1.0 : 0.0)
                                                 : (found ? 0.0 : 1.0);
    }

    // Use 1D element constraint
    return buildElementConstraint(membership, scipKeyVar, keyLb);
  }

  // Case B: Value is a variable - requires 2D matrix
  SCIP_VAR* scipValueVar = nullptr;

  // Try to extract SCIP variable - could be direct Variable reference or Expression wrapping a Variable
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(valueOperand)) {
    const Variable& valueVar = std::get<std::reference_wrapper<const Variable>>(valueOperand).get();
    scipValueVar = variableMap.at(&valueVar);
  }
  else if (std::holds_alternative<Expression>(valueOperand)) {
    // Recursively unwrap Expression(none, {Variable})
    // Navigate through 'none' operators to find the underlying Variable
    const Operand* currentOperand = &valueOperand;
    while (std::holds_alternative<Expression>(*currentOperand)) {
      const Expression& expr = std::get<Expression>(*currentOperand);
      if (expr._operator == Expression::Operator::none && expr.operands.size() == 1) {
        currentOperand = &expr.operands[0];
      } else {
        // Complex expression - not a simple variable
        throw std::runtime_error(
          "SCIPSolver: " + opName + " requires value to be a variable or constant"
        );
      }
    }

    // Should now have a Variable reference
    if (!std::holds_alternative<std::reference_wrapper<const Variable>>(*currentOperand)) {
      throw std::runtime_error("SCIPSolver: Failed to extract variable from value operand");
    }

    const Variable& valueVar = std::get<std::reference_wrapper<const Variable>>(*currentOperand).get();
    scipValueVar = variableMap.at(&valueVar);
  }
  else {
    throw std::runtime_error("SCIPSolver: Invalid value operand for " + opName);
  }

  // Get value bounds
  double valueLb = SCIPvarGetLbGlobal(scipValueVar);
  double valueUb = SCIPvarGetUbGlobal(scipValueVar);
  int numValues = (int)std::floor(valueUb) - (int)std::ceil(valueLb) + 1;

  // Build 2D membership matrix (flattened to 1D)
  // Index formula: (key - keyLb) * numValues + (value - valueLb)
  std::vector<double> membership2D(numKeys * numValues);

  for (int ki = 0; ki < numKeys; ki++) {
    int key = (int)std::ceil(keyLb) + ki;
    auto collectionResult = model.getCollection((double)key);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                               " not found in model");
    }

    const std::vector<double>& collection = collectionResult.value();

    for (int vi = 0; vi < numValues; vi++) {
      int value = (int)std::ceil(valueLb) + vi;
      bool found = std::find(collection.begin(), collection.end(), (double)value) != collection.end();

      int flatIndex = ki * numValues + vi;
      membership2D[flatIndex] = (opName == "element_of") ? (found ? 1.0 : 0.0)
                                                          : (found ? 0.0 : 1.0);
    }
  }

  // Create computed index: (key - keyLb) * numValues + (value - valueLb)
  SCIP_EXPR* keyExpr;
  SCIPcreateExprVar(scip, &keyExpr, scipKeyVar, nullptr, nullptr);

  SCIP_EXPR* valueExpr;
  SCIPcreateExprVar(scip, &valueExpr, scipValueVar, nullptr, nullptr);

  // (key - keyLb)
  SCIP_EXPR* keyMinusOffset;
  SCIPcreateExprSum(scip, &keyMinusOffset, 1, &keyExpr, nullptr, -keyLb, nullptr, nullptr);

  // (key - keyLb) * numValues
  SCIP_EXPR* keyTimesNumValues;
  double keyCoeff = (double)numValues;
  SCIPcreateExprSum(scip, &keyTimesNumValues, 1, &keyMinusOffset, &keyCoeff, 0.0, nullptr, nullptr);

  // (value - valueLb)
  SCIP_EXPR* valueMinusOffset;
  SCIPcreateExprSum(scip, &valueMinusOffset, 1, &valueExpr, nullptr, -valueLb, nullptr, nullptr);

  // (key - keyLb) * numValues + (value - valueLb)
  SCIP_EXPR* exprs[2] = {keyTimesNumValues, valueMinusOffset};
  SCIP_EXPR* computedIndexExpr;
  SCIPcreateExprSum(scip, &computedIndexExpr, 2, exprs, nullptr, 0.0, nullptr, nullptr);

  // Convert to integer variable for the computed index
  SCIP_VAR* computedIndexVar;
  std::string indexName = "computed_index_" + std::to_string(auxiliaryCounter++);
  SCIPcreateVarBasic(scip, &computedIndexVar, indexName.c_str(),
    0.0, (double)(numKeys * numValues - 1), 0.0, SCIP_VARTYPE_INTEGER);
  SCIPaddVar(scip, computedIndexVar);

  // Add constraint: computedIndexVar == computedIndexExpr
  SCIP_CONS* indexCons;
  SCIP_EXPR* indexVarExpr;
  SCIPcreateExprVar(scip, &indexVarExpr, computedIndexVar, nullptr, nullptr);

  SCIP_EXPR* indexDiff;
  SCIP_EXPR* exprsForDiff[2] = {indexVarExpr, computedIndexExpr};
  double coeffs[2] = {1.0, -1.0};
  SCIPcreateExprSum(scip, &indexDiff, 2, exprsForDiff, coeffs, 0.0, nullptr, nullptr);

  std::string consName = "index_def_" + std::to_string(auxiliaryCounter++);
  SCIPcreateConsBasicNonlinear(scip, &indexCons, consName.c_str(), indexDiff, 0.0, 0.0);
  SCIPaddCons(scip, indexCons);
  SCIPreleaseCons(scip, &indexCons);

  // Release intermediate expressions
  SCIPreleaseExpr(scip, &keyExpr);
  SCIPreleaseExpr(scip, &valueExpr);
  SCIPreleaseExpr(scip, &keyMinusOffset);
  SCIPreleaseExpr(scip, &keyTimesNumValues);
  SCIPreleaseExpr(scip, &valueMinusOffset);
  SCIPreleaseExpr(scip, &computedIndexExpr);
  SCIPreleaseExpr(scip, &indexVarExpr);
  SCIPreleaseExpr(scip, &indexDiff);

  // Use element constraint with computed index (offset 0 since we already adjusted)
  SCIP_EXPR* result = buildElementConstraint(membership2D, computedIndexVar, 0.0);
  SCIPreleaseVar(scip, &computedIndexVar);

  // Release auxiliary variable if created
  if (releaseScipKeyVar) {
    SCIPreleaseVar(scip, &scipKeyVar);
  }

  return result;
}

SCIP_EXPR* SCIPSolver::resolveCollectionItem(
    const Model& model,
    const Expression& expression
) {
  // at(index, collection(key))
  // operands[0] = custom operator index
  // operands[1] = index (constant or variable)
  // operands[2] = collection expression

  if (expression.operands.size() != 3) {
    throw std::runtime_error("SCIPSolver: at() requires exactly 2 arguments");
  }

  const Operand& indexOperand = expression.operands[1];
  const Expression& collectionExpr = std::get<Expression>(expression.operands[2]);

  // Extract key from collection(key)
  if (collectionExpr.operands.size() != 1) {
    throw std::runtime_error("SCIPSolver: collection() must have exactly 1 argument");
  }

  const Operand& keyOperand = collectionExpr.operands[0];

  // Handle constant key
  auto constantKeyOpt = extractConstant(keyOperand);
  if (constantKeyOpt.has_value()) {
    double constantKey = constantKeyOpt.value();
    auto collectionResult = model.getCollection(constantKey);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(constantKey) +
                               " not found in model");
    }

    const std::vector<double>& collection = collectionResult.value();

    // With constant key, just get element at index
    auto constantIndexOpt = extractConstant(indexOperand);
    if (constantIndexOpt.has_value()) {
      size_t index = (size_t)constantIndexOpt.value();
      if (index < 1 || index > collection.size()) {
        throw std::runtime_error("SCIPSolver: Index out of bounds");
      }

      SCIP_EXPR* resultExpr;
      SCIPcreateExprValue(scip, &resultExpr, collection[index - 1], nullptr, nullptr);
      return resultExpr;
    } else {
      throw std::runtime_error(
        "SCIPSolver: at() with constant key requires constant index"
      );
    }
  }

  // Unwrap Expression(none, {operand}) to get the actual operand
  const Operand* actualKeyOperand = &keyOperand;
  if (std::holds_alternative<Expression>(keyOperand)) {
    const Expression& keyExpr = std::get<Expression>(keyOperand);
    if (keyExpr._operator == Expression::Operator::none && keyExpr.operands.size() == 1) {
      actualKeyOperand = &keyExpr.operands[0];
    }
  }

  SCIP_VAR* scipKeyVar = nullptr;
  bool releaseScipKeyVar = false;  // Track if we need to release the auxiliary variable

  // Handle variable key
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
    const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand).get();
    auto it = variableMap.find(&keyVar);
    if (it == variableMap.end()) {
      throw std::runtime_error("SCIPSolver: Collection key variable not found");
    }
    scipKeyVar = it->second;
  }
  // Handle IndexedVariable key
  else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
    const IndexedVariable& idxVar = std::get<IndexedVariable>(*actualKeyOperand);

    // Build expression for the indexed variable
    SCIP_EXPR* keyScipExpr = buildExpression(model, *actualKeyOperand);

    // Get bounds from the IndexedVariables container's variables
    const IndexedVariables& container = idxVar.container.get();
    double minBound = std::numeric_limits<double>::max();
    double maxBound = std::numeric_limits<double>::lowest();

    for (const auto& var : container) {
      auto it = variableMap.find(&var);
      if (it != variableMap.end()) {
        double lb = SCIPvarGetLbGlobal(it->second);
        double ub = SCIPvarGetUbGlobal(it->second);
        minBound = std::min(minBound, lb);
        maxBound = std::max(maxBound, ub);
      }
    }

    // Create auxiliary variable for the collection key
    size_t auxId = auxiliaryCounter++;
    std::string auxName = "aux_collection_key_" + std::to_string(auxId);
    SCIP_VAR* auxKeyVar;
    SCIPcreateVarBasic(scip, &auxKeyVar, auxName.c_str(), minBound, maxBound,
                       0.0, SCIP_VARTYPE_INTEGER);
    SCIPaddVar(scip, auxKeyVar);

    // Add constraint: auxKeyVar == keyScipExpr
    SCIP_EXPR* auxExpr;
    SCIPcreateExprVar(scip, &auxExpr, auxKeyVar, nullptr, nullptr);

    SCIP_EXPR* diffExprs[] = { auxExpr, keyScipExpr };
    double coeffs[] = { 1.0, -1.0 };
    SCIP_EXPR* diffExpr;
    SCIPcreateExprSum(scip, &diffExpr, 2, diffExprs, coeffs, 0.0, nullptr, nullptr);

    SCIP_CONS* auxCons;
    std::string consName = "aux_collection_key_cons_" + std::to_string(auxId);
    SCIPcreateConsBasicNonlinear(scip, &auxCons, consName.c_str(), diffExpr, 0.0, 0.0);
    SCIPaddCons(scip, auxCons);
    SCIPreleaseCons(scip, &auxCons);
    SCIPreleaseExpr(scip, &diffExpr);
    SCIPreleaseExpr(scip, &auxExpr);
    SCIPreleaseExpr(scip, &keyScipExpr);

    scipKeyVar = auxKeyVar;
    releaseScipKeyVar = true;  // Mark for cleanup
  }
  else {
    throw std::runtime_error(
      "SCIPSolver: collection() key must be a variable or constant"
    );
  }

  // Get key bounds
  double keyLb = SCIPvarGetLbGlobal(scipKeyVar);
  double keyUb = SCIPvarGetUbGlobal(scipKeyVar);
  int numKeys = (int)std::floor(keyUb) - (int)std::ceil(keyLb) + 1;

  // Case A: Index is a constant
  if (std::holds_alternative<double>(indexOperand)) {
    size_t index = (size_t)std::get<double>(indexOperand);

    // Validate index is a positive integer (1-based)
    if (index < 1) {
      throw std::runtime_error("SCIPSolver: Index must be >= 1 (1-based indexing)");
    }

    // Pre-compute element at this index for each collection
    std::vector<double> elements(numKeys);

    for (int i = 0; i < numKeys; i++) {
      int key = (int)std::ceil(keyLb) + i;
      auto collectionResult = model.getCollection((double)key);
      if (!collectionResult) {
        throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                                 " not found in model");
      }

      const std::vector<double>& collection = collectionResult.value();

      // Validate index (1-based)
      if (index > collection.size()) {
        throw std::runtime_error(
          "SCIPSolver: Index " + std::to_string(index) +
          " out of bounds for collection at key " + std::to_string(key) +
          " (size: " + std::to_string(collection.size()) + ")"
        );
      }

      elements[i] = collection[index - 1];  // Convert to 0-based
    }

    // Use 1D element constraint
    SCIP_EXPR* result = buildElementConstraint(elements, scipKeyVar, keyLb);

    // Release auxiliary variable if created
    if (releaseScipKeyVar) {
      SCIPreleaseVar(scip, &scipKeyVar);
    }

    return result;
  }
  // Case B: Index is a variable - requires 2D matrix
  SCIP_VAR* scipIndexVar = nullptr;

  // Try to extract SCIP variable - could be direct Variable reference or Expression wrapping a Variable
  if (std::holds_alternative<std::reference_wrapper<const Variable>>(indexOperand)) {
    const Variable& indexVar = std::get<std::reference_wrapper<const Variable>>(indexOperand).get();
    scipIndexVar = variableMap.at(&indexVar);
  }
  else if (std::holds_alternative<Expression>(indexOperand)) {
    // Recursively unwrap Expression(none, {Variable})
    // Navigate through 'none' operators to find the underlying Variable
    const Operand* currentOperand = &indexOperand;
    while (std::holds_alternative<Expression>(*currentOperand)) {
      const Expression& expr = std::get<Expression>(*currentOperand);
      if (expr._operator == Expression::Operator::none && expr.operands.size() == 1) {
        currentOperand = &expr.operands[0];
      } else {
        // Complex expression - not a simple variable
        throw std::runtime_error(
          "SCIPSolver: at() requires index to be a variable or constant"
        );
      }
    }

    // Should now have a Variable reference
    if (!std::holds_alternative<std::reference_wrapper<const Variable>>(*currentOperand)) {
      throw std::runtime_error("SCIPSolver: Failed to extract variable from index operand");
    }

    const Variable& indexVar = std::get<std::reference_wrapper<const Variable>>(*currentOperand).get();
    scipIndexVar = variableMap.at(&indexVar);
  }
  else {
    throw std::runtime_error("SCIPSolver: Invalid index operand for at()");
  }

  // Get index bounds (1-based)
  double indexLb = SCIPvarGetLbGlobal(scipIndexVar);
  double indexUb = SCIPvarGetUbGlobal(scipIndexVar);
  int numIndices = (int)std::floor(indexUb) - (int)std::ceil(indexLb) + 1;

  // Validate that all indices are within bounds for all collections
  for (int ki = 0; ki < numKeys; ki++) {
    int key = (int)std::ceil(keyLb) + ki;
    auto collectionResult = model.getCollection((double)key);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                               " not found in model");
    }

    const std::vector<double>& collection = collectionResult.value();
    size_t maxIndex = (size_t)std::ceil(indexUb);

    if (maxIndex > collection.size()) {
      throw std::runtime_error(
        "SCIPSolver: Index upper bound " + std::to_string(maxIndex) +
        " exceeds collection size " + std::to_string(collection.size()) +
        " for key " + std::to_string(key)
      );
    }
  }

  // Build 2D matrix of elements (flattened to 1D)
  // Index formula: (key - keyLb) * numIndices + (index - indexLb)
  std::vector<double> elements2D(numKeys * numIndices);
  double minElement = std::numeric_limits<double>::max();
  double maxElement = std::numeric_limits<double>::lowest();

  for (int ki = 0; ki < numKeys; ki++) {
    int key = (int)std::ceil(keyLb) + ki;
    auto collectionResult = model.getCollection((double)key);
    if (!collectionResult) {
      throw std::runtime_error("SCIPSolver: Collection key " + std::to_string(key) +
                               " not found in model");
    }
    const std::vector<double>& collection = collectionResult.value();

    for (int ii = 0; ii < numIndices; ii++) {
      size_t index = (size_t)std::ceil(indexLb) + ii;  // 1-based

      // Validate index
      if (index < 1 || index > collection.size()) {
        throw std::runtime_error(
          "SCIPSolver: Index " + std::to_string(index) +
          " out of bounds for collection at key " + std::to_string(key)
        );
      }

      int flatIndex = ki * numIndices + ii;
      double element = collection[index - 1];  // Convert to 0-based
      elements2D[flatIndex] = element;

      minElement = std::min(minElement, element);
      maxElement = std::max(maxElement, element);
    }
  }

  // Create computed index: (key - keyLb) * numIndices + (index - indexLb)
  SCIP_EXPR* keyExpr;
  SCIPcreateExprVar(scip, &keyExpr, scipKeyVar, nullptr, nullptr);

  SCIP_EXPR* indexExpr;
  SCIPcreateExprVar(scip, &indexExpr, scipIndexVar, nullptr, nullptr);

  // (key - keyLb)
  SCIP_EXPR* keyMinusOffset;
  SCIPcreateExprSum(scip, &keyMinusOffset, 1, &keyExpr, nullptr, -keyLb, nullptr, nullptr);

  // (key - keyLb) * numIndices
  SCIP_EXPR* keyTimesNumIndices;
  double keyCoeff = (double)numIndices;
  SCIPcreateExprSum(scip, &keyTimesNumIndices, 1, &keyMinusOffset, &keyCoeff, 0.0, nullptr, nullptr);

  // (index - indexLb)
  SCIP_EXPR* indexMinusOffset;
  SCIPcreateExprSum(scip, &indexMinusOffset, 1, &indexExpr, nullptr, -indexLb, nullptr, nullptr);

  // (key - keyLb) * numIndices + (index - indexLb)
  SCIP_EXPR* exprs[2] = {keyTimesNumIndices, indexMinusOffset};
  SCIP_EXPR* computedIndexExpr;
  SCIPcreateExprSum(scip, &computedIndexExpr, 2, exprs, nullptr, 0.0, nullptr, nullptr);

  // Convert to integer variable for the computed index
  SCIP_VAR* computedIndexVar;
  std::string indexName = "computed_index_" + std::to_string(auxiliaryCounter++);
  SCIPcreateVarBasic(scip, &computedIndexVar, indexName.c_str(),
    0.0, (double)(numKeys * numIndices - 1), 0.0, SCIP_VARTYPE_INTEGER);
  SCIPaddVar(scip, computedIndexVar);

  // Add constraint: computedIndexVar == computedIndexExpr
  SCIP_CONS* indexCons;
  SCIP_EXPR* indexVarExpr;
  SCIPcreateExprVar(scip, &indexVarExpr, computedIndexVar, nullptr, nullptr);

  SCIP_EXPR* indexDiff;
  SCIP_EXPR* exprsForDiff[2] = {indexVarExpr, computedIndexExpr};
  double coeffs[2] = {1.0, -1.0};
  SCIPcreateExprSum(scip, &indexDiff, 2, exprsForDiff, coeffs, 0.0, nullptr, nullptr);

  std::string consName = "index_def_" + std::to_string(auxiliaryCounter++);
  SCIPcreateConsBasicNonlinear(scip, &indexCons, consName.c_str(), indexDiff, 0.0, 0.0);
  SCIPaddCons(scip, indexCons);
  SCIPreleaseCons(scip, &indexCons);

  // Release intermediate expressions
  SCIPreleaseExpr(scip, &keyExpr);
  SCIPreleaseExpr(scip, &indexExpr);
  SCIPreleaseExpr(scip, &keyMinusOffset);
  SCIPreleaseExpr(scip, &keyTimesNumIndices);
  SCIPreleaseExpr(scip, &indexMinusOffset);
  SCIPreleaseExpr(scip, &computedIndexExpr);
  SCIPreleaseExpr(scip, &indexVarExpr);
  SCIPreleaseExpr(scip, &indexDiff);

  // Use element constraint with computed index (offset 0 since we already adjusted)
  SCIP_EXPR* result = buildElementConstraint(elements2D, computedIndexVar, 0.0);
  SCIPreleaseVar(scip, &computedIndexVar);

  // Release auxiliary variable if created
  if (releaseScipKeyVar) {
    SCIPreleaseVar(scip, &scipKeyVar);
  }

  return result;
}

} // namespace CP
