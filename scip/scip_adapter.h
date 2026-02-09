#pragma once

#include "solver.h"
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
  const std::unordered_map<const Variable*, SCIP_VAR*>& getVarMap() const { return varMap_; }

private:
  void addVariables(const Model& model);
  void addIndexedVariables(const Model& model);
  void addSequences(const Model& model);
  void addObjective(const Model& model);
  void addConstraints(const Model& model);

  std::expected<SCIP_EXPR*, std::string> buildExpr(const Operand& operand);

  // Helper functions for constraints
  void addSequenceConstraints(const std::string& seqName, const std::vector<SCIP_VAR*>& seqVars);
  SCIP_EXPR* addElementConstraint(const std::string& name, const std::vector<SCIP_VAR*>& arrayVars,
                                   SCIP_VAR* indexVar, SCIP_VAR* resultVar);

  SCIP* scip_ = nullptr;
  std::unordered_map<const Variable*, SCIP_VAR*> varMap_;
  double epsilon_;
};

} // namespace CP
