#pragma once

#include "../solver.h"
#include <scip/scip.h>
#include <unordered_map>

namespace CP {

class SCIPSolver : public Solver {
public:
  SCIPSolver(const Model& model, double epsilon = 1e-6);
  ~SCIPSolver() override;

  std::expected<Solution, std::string> solve(const Model& model) override;
  std::string getName() const override { return "SCIP"; }

  // For testing: expose SCIP state
  SCIP* getScip() const { return scip_; }
  const std::unordered_map<const Variable*, SCIP_VAR*>& getVariableMap() const { return variableMap_; }

private:
  void addSequences(const Model& model);
  void addVariables(const Model& model);
  void addIndexedVariables(const Model& model);
  void addObjective(const Model& model);
  void addConstraints(const Model& model);

  std::expected<SCIP_EXPR*, std::string> buildExpr(const Operand& operand);

  // Helper functions for constraints
  void addSequenceConstraints(const std::string& sequenceName, const std::vector<SCIP_VAR*>& seqVars);
  SCIP_EXPR* addIndexingConstraints(const std::string& name, const std::vector<SCIP_VAR*>& arrayVars, SCIP_VAR* indexVar, SCIP_VAR* resultVar);
  SCIP_EXPR* createBoolExpr(SCIP_EXPR* expr);

  SCIP* scip_ = nullptr;
  std::unordered_map<const Variable*, SCIP_VAR*> variableMap_;
  double epsilon_;
};

} // namespace CP
