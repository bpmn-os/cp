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

    std::expected<Solution, std::string> solve(const Model& model) override;
    std::expected<Solution, std::string> solve(const Model& model, double timeLimit);
    std::string getName() const override { return "Hexaly"; }

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

    std::unique_ptr<hexaly::HexalyOptimizer> optimizer;
    hexaly::HxModel hxModel;
    unsigned int precision;  // Number of decimal places for solution rounding

    // Maps CP variables to Hexaly expressions
    std::unordered_map<const Variable*, hexaly::HxExpression> expressionMap;

    // Maps sequences to list expressions
    std::unordered_map<const Sequence*, hexaly::HxExpression> sequenceMap;
};

} // namespace CP
