#pragma once

#include "../solver.h"
#include <optimizer/hexalyoptimizer.h>
#include <unordered_map>
#include <memory>

namespace CP {

class HexalySolver : public Solver {
public:
    HexalySolver(const Model& model, unsigned int precision = 4);
    ~HexalySolver() override;

    std::string getName() const override { return "Hexaly"; }

    Result solve(double timeLimit = std::numeric_limits<double>::infinity()) override;
    void stop() override;

    // For testing
    hexaly::HexalyOptimizer& getOptimizer() { return *optimizer; }

private:
    void addSequences(const Model& model);
    void addAllVariables(const Model& model);
    void addObjective(const Model& model);
    void addConstraints(const Model& model);

    hexaly::HxExpression buildExpression(const Model& model, const Operand& operand);
    hexaly::HxExpression buildCustomOperator(const Model& model, const Expression& expr);
    hexaly::HxExpression boolify(hexaly::HxExpression expr);
    hexaly::HxExpression round(hexaly::HxExpression expr);

    // Collection helpers
    hexaly::HxExpression resolveCollectionOperation(const Model& model, const Expression& expr, const std::string& opName);
    hexaly::HxExpression resolveCollectionMembership(const Model& model, const Expression& expr, const std::string& opName);
    hexaly::HxExpression resolveCollectionItem(const Model& model, const Expression& expr);
    hexaly::HxExpression resolveCollectionAccess(const Model& model, const Expression& expr);

private:
    friend class IterationCallback;
    // Called by callback on each iteration
    void notifyIteration();
    // Called by callback when solution may have improved
    void checkForNewSolution();

    std::unique_ptr<hexaly::HexalyOptimizer> optimizer;
    hexaly::HxModel hxModel;
    unsigned int precision;  // Number of decimal places for solution rounding
    std::atomic<bool> stopped_{false};  // Track if stop() was called
    double lastBestObjective_ = std::numeric_limits<double>::quiet_NaN();  // Track best objective for callback
    std::unique_ptr<hexaly::HxCallback> solutionCallback_;  // Callback for solution notifications
    hexaly::HxExpression objectiveExpr_;  // Stored objective for solution comparison

    // Maps CP variables to Hexaly expressions
    std::unordered_map<const Variable*, hexaly::HxExpression> expressionMap;

    // Maps sequences to list expressions
    std::unordered_map<const Sequence*, hexaly::HxExpression> sequenceMap;
};

} // namespace CP
