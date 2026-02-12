#pragma once

#include "../solver.h"
#include <scip/scip.h>
#include <unordered_map>
#include <limits>

namespace CP {

class SCIPSolver : public Solver {
public:
  SCIPSolver(const Model& model, unsigned int precision = 4);
  ~SCIPSolver() override;

  std::expected<Solution, std::string> solve(const Model& model) override;
  std::expected<Solution, std::string> solve(const Model& model, double timeLimit);
  std::string getName() const override { return "SCIP"; }

  // For testing: expose SCIP state
  SCIP* getScip() const { return scip; }
  const std::unordered_map<const Variable*, SCIP_VAR*>& getVariableMap() const { return variableMap; }

private:
  void addSequences(const Model& model);
  void addVariables(const Model& model);
  void addIndexedVariables(const Model& model);
  void addDeducedConstraints(const Model& model);
  void addObjective(const Model& model);
  void addConstraints(const Model& model);

  SCIP_EXPR* buildExpression(const Operand& operand);

  // Helper functions for constraints
  void addSequenceConstraints(const std::string& sequenceName, const std::vector<SCIP_VAR*>& sequenceVariables);
  SCIP_EXPR* addIndexingConstraints(const std::string& name, const std::vector<SCIP_VAR*>& arrayVars, SCIP_VAR* indexVar, SCIP_VAR* resultVar, int indexOffset = 0);
  SCIP_EXPR* boolify(SCIP_EXPR* scipExpr);

  SCIP* scip = nullptr;
  std::unordered_map<const Variable*, SCIP_VAR*> variableMap;
  unsigned int precision;  // Number of decimal places for CP solution rounding
  double epsilon;      // SCIP's feasibility tolerance (numerics/feastol) for constraint formulations
  size_t auxiliaryCounter = 0;
};

} // namespace CP
