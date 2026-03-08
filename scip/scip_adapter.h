#pragma once

#include "../solver.h"
#include <scip/scip.h>
#include <unordered_map>
#include <memory>

namespace CP {

struct SolEventData;  // Forward declaration

class SCIPSolver : public Solver {
public:
  SCIPSolver(const Model& model, unsigned int precision = 4);
  ~SCIPSolver() override;

  std::string getName() const override { return "SCIP"; }

  Result solve(double timeLimit = std::numeric_limits<double>::infinity()) override;
  void stop() override;

  void fix(const Variable& variable, double value) override;
  void fix(const Sequence& sequence, const std::vector<int>& values) override;
  void unfix() override;

  // Called by event handler when new best solution is found
  void notifyNewSolution();
  // Called by event handler on each iteration (node)
  void notifyIteration();
  // Check if listeners are registered (for event handler init)
  bool hasSolutionListener() const { return static_cast<bool>(onSolution); }
  bool hasIterationListener() const { return static_cast<bool>(onIteration); }

  // For testing: expose SCIP state
  SCIP* getScip() const { return scip; }
  const std::unordered_map<const Variable*, SCIP_VAR*>& getVariableMap() const { return variableMap; }

private:
  void ensureProblemStage();  // Returns SCIP to PROBLEM stage if needed
  void addSequences(const Model& model);
  void addVariables(const Model& model);
  void addIndexedVariables(const Model& model);
  void addDeducedConstraints(const Model& model);
  void addObjective(const Model& model);
  void addConstraints(const Model& model);

  SCIP_EXPR* buildExpression(const Model& model, const Operand& operand);

  // Helper functions for constraints
  void addSequenceConstraints(const std::string& sequenceName, const std::vector<SCIP_VAR*>& sequenceVariables);
  SCIP_EXPR* addIndexingConstraints(const std::string& name, const std::vector<SCIP_VAR*>& arrayVars, SCIP_VAR* indexVar, SCIP_VAR* resultVar, int indexOffset = 0);
  SCIP_EXPR* boolify(SCIP_EXPR* scipExpr);

  // Collection operation helpers
  SCIP_EXPR* resolveCollectionOperation(
    const Model& model,
    const Expression& expression,
    const std::string& opName
  );

  SCIP_EXPR* resolveCollectionMembership(
    const Model& model,
    const Expression& expression,
    const std::string& opName
  );

  SCIP_EXPR* resolveCollectionItem(
    const Model& model,
    const Expression& expression
  );

  SCIP_EXPR* buildElementConstraint(
    const std::vector<double>& values,
    SCIP_VAR* indexVar,
    double indexOffset
  );

  SCIP* scip = nullptr;
  std::unordered_map<const Variable*, SCIP_VAR*> variableMap;
  unsigned int precision;  // Number of decimal places for CP solution rounding
  double epsilon;      // SCIP's feasibility tolerance (numerics/feastol) for constraint formulations
  size_t auxiliaryCounter = 0;

  // Event handlers for callbacks
  std::unique_ptr<SolEventData> solutionEventData_;
  std::unique_ptr<SolEventData> iterationEventData_;
  SCIP_EVENTHDLR* solEventhdlr_ = nullptr;
  SCIP_EVENTHDLR* iterEventhdlr_ = nullptr;

  // Track last notified solution objective to avoid duplicate notifications
  double lastBestObjective_ = std::numeric_limits<double>::quiet_NaN();

  // Fix/unfix state - track fixed variables to restore bounds on unfix
  std::vector<std::pair<const Variable*, double>> fixedVariables_;
};

} // namespace CP
