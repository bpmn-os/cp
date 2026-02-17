#include "hexaly_adapter.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <numeric>
#include <algorithm>
#include <unordered_set>

namespace CP {

HexalySolver::HexalySolver(const Model& model)
    : optimizer(std::make_unique<hexaly::HexalyOptimizer>())
    , hxModel(optimizer->getModel())
{
    optimizer->getParam().setVerbosity(0);  // Suppress output by default

    addSequences(model);
    addAllVariables(model);
    addObjective(model);
    addConstraints(model);

    hxModel.close();
}

HexalySolver::~HexalySolver() {
    // Explicitly reset optimizer to release Hexaly license token
    optimizer.reset();
}

void HexalySolver::addSequences(const Model& model) {
    for (const auto& sequence : model.getSequences()) {
        // Hexaly uses list variables for permutations
        // Domain is {0, ..., n-1}, CP uses {1, ..., n}
        int n = static_cast<int>(sequence.variables.size());
        hexaly::HxExpression listVar = hxModel.listVar(n);

        // Ensure all elements are used (full permutation)
        hxModel.constraint(hxModel.count(listVar) == n);

        sequenceMap[&sequence] = listVar;

        // Create expressions for individual sequence variables
        // sequence.variables[i] gives the value at position i
        // In Hexaly list: listVar[i] gives 0-based value, we need 1-based
        for (int i = 0; i < n; i++) {
            // at(listVar, i) returns the element at position i (0-based value)
            // Add 1 to convert to 1-based
            hexaly::HxExpression posExpr = hxModel.at(listVar, i) + 1;
            expressionMap[&sequence.variables[i]] = posExpr;
        }
    }
}

void HexalySolver::addAllVariables(const Model& model) {
    for (const Variable& variable : model.getAllVariables()) {
        hexaly::HxExpression expr;

        if (variable.deducedFrom) {
            // Hexaly: expressions are first-class!
            // No extra constraint needed - just use the expression directly
            expr = buildExpression(model, *variable.deducedFrom);
        }
        else {
            // Create decision variable
            double lb = variable.lowerBound;
            double ub = variable.upperBound;

            switch (variable.type) {
                case Variable::Type::BOOLEAN:
                    if (lb == 1 && ub == 1) {
                        // Fixed to true
                        expr = hxModel.createConstant(static_cast<hexaly::hxint>(1));
                    }
                    else if (lb == 0 && ub == 0) {
                        // Fixed to false
                        expr = hxModel.createConstant(static_cast<hexaly::hxint>(0));
                    }
                    else {
                        expr = hxModel.boolVar();
                    }
                    break;
                case Variable::Type::INTEGER: {
                    hexaly::hxint intLb = (lb == std::numeric_limits<double>::lowest())
                        ? HX_INT_MIN
                        : static_cast<hexaly::hxint>(lb);
                    hexaly::hxint intUb = (ub == std::numeric_limits<double>::max())
                        ? HX_INT_MAX
                        : static_cast<hexaly::hxint>(ub);
                    expr = hxModel.intVar(intLb, intUb);
                    break;
                }
                case Variable::Type::REAL:
                    expr = hxModel.floatVar(lb, ub);
                    break;
            }
        }

        expressionMap[&variable] = expr;
    }
}

void HexalySolver::addObjective(const Model& model) {
    if (model.getObjectiveSense() == Model::ObjectiveSense::FEASIBLE) {
        // Hexaly requires at least one objective - use dummy constant
        hxModel.minimize(hxModel.createConstant(0.0));
        return;
    }

    hexaly::HxExpression objExpr = buildExpression(model, model.getObjective());

    switch (model.getObjectiveSense()) {
        case Model::ObjectiveSense::MINIMIZE:
            hxModel.minimize(objExpr);
            break;
        case Model::ObjectiveSense::MAXIMIZE:
            hxModel.maximize(objExpr);
            break;
        default:
            break;
    }
}

void HexalySolver::addConstraints(const Model& model) {
    for (const auto& constraint : model.getConstraints()) {
        hexaly::HxExpression constraintExpr = buildExpression(model, constraint);
        hxModel.constraint(constraintExpr);
    }
}

hexaly::HxExpression HexalySolver::boolify(hexaly::HxExpression expr) {
    // Hexaly requires boolean operands for logical operators
    // Convert non-boolean to boolean: (expr != 0)
    return (expr != hxModel.createConstant(0.0));
}

hexaly::HxExpression HexalySolver::round(hexaly::HxExpression expr) {
    // Hexaly requires integer operands for at() indices
    // Convert float to integer using round()
    return hxModel.round(expr);
}

hexaly::HxExpression HexalySolver::buildExpression(const Model& model, const Operand& operand) {
    using Op = Expression::Operator;

    // Handle constant
    if (std::holds_alternative<double>(operand)) {
        return hxModel.createConstant(std::get<double>(operand));
    }

    // Handle variable reference
    if (std::holds_alternative<std::reference_wrapper<const Variable>>(operand)) {
        const Variable& var = std::get<std::reference_wrapper<const Variable>>(operand);
        return expressionMap.at(&var);
    }

    // Handle indexed variable (element constraint)
    if (std::holds_alternative<IndexedVariable>(operand)) {
        const IndexedVariable& iv = std::get<IndexedVariable>(operand);
        const IndexedVariables& container = iv.container.get();

        // Build array from all variables in container
        std::vector<hexaly::HxExpression> arrayExprs;
        for (size_t i = 0; i < container.size(); i++) {
            arrayExprs.push_back(expressionMap.at(&container[i]));
        }

        // Build index expression (CP uses 0-based indexing for IndexedVariables)
        hexaly::HxExpression indexExpr = expressionMap.at(&iv.index.get());

        return hxModel.at(hxModel.array(arrayExprs.begin(), arrayExprs.end()), round(indexExpr));
    }

    // Handle expression
    const Expression& expr = std::get<Expression>(operand);

    switch (expr._operator) {
        case Op::none:
            return buildExpression(model, expr.operands[0]);

        case Op::negate:
            return hxModel.createConstant(static_cast<hexaly::hxint>(-1)) * buildExpression(model, expr.operands[0]);

        case Op::logical_not: {
            auto inner = buildExpression(model, expr.operands[0]);
            return !boolify(inner);
        }

        case Op::logical_and: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return (boolify(lhs) && boolify(rhs));
        }

        case Op::logical_or: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return (boolify(lhs) || boolify(rhs));
        }

        case Op::add: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs + rhs;
        }

        case Op::subtract: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs - rhs;
        }

        case Op::multiply: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs * rhs;
        }

        case Op::divide: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs / rhs;
        }

        case Op::less_than: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs < rhs;
        }

        case Op::less_or_equal: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs <= rhs;
        }

        case Op::greater_than: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs > rhs;
        }

        case Op::greater_or_equal: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs >= rhs;
        }

        case Op::equal: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs == rhs;
        }

        case Op::not_equal: {
            auto lhs = buildExpression(model, expr.operands[0]);
            auto rhs = buildExpression(model, expr.operands[1]);
            return lhs != rhs;
        }

        case Op::custom:
            return buildCustomOperator(model, expr);

        case Op::collection:
            // Collection wrapper - unwrap and return inner expression
            if (expr.operands.size() == 1) {
                return buildExpression(model, expr.operands[0]);
            }
            throw std::runtime_error("HexalySolver: collection() must have exactly 1 argument");

        case Op::at:
            // Collection access: Collection(key)[index]
            return resolveCollectionAccess(model, expr);

        default:
            throw std::runtime_error("Unsupported operator in Hexaly adapter");
    }
}

hexaly::HxExpression HexalySolver::buildCustomOperator(const Model& model, const Expression& expr) {
    size_t opIndex = std::get<size_t>(expr.operands[0]);
    const std::string& opName = Expression::customOperators[opIndex];

    // Check if any operand is a collection() expression
    bool hasCollection = false;
    for (size_t i = 1; i < expr.operands.size(); i++) {
        if (std::holds_alternative<Expression>(expr.operands[i])) {
            const Expression& operandExpr = std::get<Expression>(expr.operands[i]);
            if (operandExpr._operator == Expression::Operator::collection) {
                hasCollection = true;
                break;
            }
        }
    }

    // Route collection operations to specialized handlers
    if (hasCollection) {
        static const std::unordered_set<std::string> supportedCollectionOps = {
            "count", "sum", "avg", "max", "min", "element_of", "not_element_of", "at"
        };
        if (supportedCollectionOps.find(opName) == supportedCollectionOps.end()) {
            throw std::runtime_error(
                "HexalySolver: Custom operator '" + opName + "' cannot take collection() expressions. "
                "Only count, sum, avg, max, min, element_of, not_element_of, and at can process collections."
            );
        }

        if (opName == "count" || opName == "sum" || opName == "avg" || opName == "max" || opName == "min") {
            return resolveCollectionOperation(model, expr, opName);
        }
        else if (opName == "element_of" || opName == "not_element_of") {
            return resolveCollectionMembership(model, expr, opName);
        }
        else if (opName == "at") {
            return resolveCollectionItem(model, expr);
        }
    }

    // Non-collection operations
    if (opName == "min") {
        std::vector<hexaly::HxExpression> args;
        for (size_t i = 1; i < expr.operands.size(); i++) {
            args.push_back(buildExpression(model, expr.operands[i]));
        }
        return hxModel.min(hxModel.array(args.begin(), args.end()));
    }

    if (opName == "max") {
        std::vector<hexaly::HxExpression> args;
        for (size_t i = 1; i < expr.operands.size(); i++) {
            args.push_back(buildExpression(model, expr.operands[i]));
        }
        return hxModel.max(hxModel.array(args.begin(), args.end()));
    }

    if (opName == "abs") {
        return hxModel.abs(buildExpression(model, expr.operands[1]));
    }

    if (opName == "pow") {
        auto base = buildExpression(model, expr.operands[1]);
        auto exp = buildExpression(model, expr.operands[2]);
        return hxModel.pow(base, exp);
    }

    if (opName == "sum") {
        std::vector<hexaly::HxExpression> args;
        for (size_t i = 1; i < expr.operands.size(); i++) {
            args.push_back(buildExpression(model, expr.operands[i]));
        }
        return hxModel.sum(hxModel.array(args.begin(), args.end()));
    }

    if (opName == "avg") {
        std::vector<hexaly::HxExpression> args;
        for (size_t i = 1; i < expr.operands.size(); i++) {
            args.push_back(buildExpression(model, expr.operands[i]));
        }
        // avg = sum / count
        auto sumExpr = hxModel.sum(hxModel.array(args.begin(), args.end()));
        auto countVal = hxModel.createConstant(static_cast<double>(args.size()));
        return sumExpr / countVal;
    }

    if (opName == "count") {
        // count simply returns the number of arguments (as integer)
        hexaly::hxint count = static_cast<hexaly::hxint>(expr.operands.size() - 1);
        return hxModel.createConstant(count);
    }

    if (opName == "if_then_else") {
        auto cond = buildExpression(model, expr.operands[1]);
        auto thenExpr = buildExpression(model, expr.operands[2]);
        auto elseExpr = buildExpression(model, expr.operands[3]);
        return hxModel.iif(boolify(cond), thenExpr, elseExpr);
    }

    if (opName == "n_ary_if") {
        // Build nested iif - last operand is default
        hexaly::HxExpression result = buildExpression(model, expr.operands.back());

        // Process pairs (condition, value) from back to front
        for (int i = static_cast<int>(expr.operands.size()) - 3; i >= 1; i -= 2) {
            auto cond = buildExpression(model, expr.operands[i]);
            auto val = buildExpression(model, expr.operands[i + 1]);
            result = hxModel.iif(boolify(cond), val, result);
        }
        return result;
    }

    if (opName == "at") {
        // Element constraint with explicit values (non-collection)
        // CP uses 1-based indexing, Hexaly uses 0-based
        // Hexaly requires integer index - convert double constants to integer
        hexaly::HxExpression zeroBasedIndex;
        if (std::holds_alternative<double>(expr.operands[1])) {
            hexaly::hxint intIndex = static_cast<hexaly::hxint>(std::get<double>(expr.operands[1])) - 1;
            zeroBasedIndex = hxModel.createConstant(intIndex);
        }
        else {
            auto index = buildExpression(model, expr.operands[1]);
            zeroBasedIndex = round(index) - hxModel.createConstant(static_cast<hexaly::hxint>(1));
        }
        std::vector<hexaly::HxExpression> values;
        for (size_t i = 2; i < expr.operands.size(); i++) {
            values.push_back(buildExpression(model, expr.operands[i]));
        }
        return hxModel.at(hxModel.array(values.begin(), values.end()), zeroBasedIndex);
    }

    throw std::runtime_error("Unknown custom operator: " + opName);
}

// Collection operation: count/sum/avg/min/max(Collection(key))
hexaly::HxExpression HexalySolver::resolveCollectionOperation(
    const Model& model, const Expression& expr, const std::string& opName
) {
    // Extract collection expression: operands[1] = collection(key)
    const Expression& collectionExpr = std::get<Expression>(expr.operands[1]);

    if (collectionExpr.operands.size() != 1) {
        throw std::runtime_error("HexalySolver: collection() must have exactly 1 argument");
    }

    const Operand& keyOperand = collectionExpr.operands[0];

    // Case 1: Constant key - compute result directly
    if (std::holds_alternative<double>(keyOperand)) {
        double constantKey = std::get<double>(keyOperand);
        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        double result;
        if (opName == "count") {
            result = static_cast<double>(collection.size());
        }
        else if (opName == "sum") {
            result = std::accumulate(collection.begin(), collection.end(), 0.0);
        }
        else if (opName == "avg") {
            if (collection.empty()) {
                throw std::runtime_error("HexalySolver: avg() is undefined for empty collection");
            }
            result = std::accumulate(collection.begin(), collection.end(), 0.0) / collection.size();
        }
        else if (opName == "max") {
            if (collection.empty()) {
                throw std::runtime_error("HexalySolver: max() is undefined for empty collection");
            }
            result = *std::max_element(collection.begin(), collection.end());
        }
        else if (opName == "min") {
            if (collection.empty()) {
                throw std::runtime_error("HexalySolver: min() is undefined for empty collection");
            }
            result = *std::min_element(collection.begin(), collection.end());
        }
        else {
            throw std::runtime_error("HexalySolver: Unknown collection operation: " + opName);
        }

        return hxModel.createConstant(result);
    }

    // Unwrap Expression(none, {operand}) to get the actual operand
    const Operand* actualKeyOperand = &keyOperand;
    if (std::holds_alternative<Expression>(keyOperand)) {
        const Expression& keyExpr = std::get<Expression>(keyOperand);
        if (keyExpr._operator == Expression::Operator::none && keyExpr.operands.size() == 1) {
            actualKeyOperand = &keyExpr.operands[0];
        }
    }

    // Case 2: Variable or IndexedVariable key - build array lookup
    hexaly::HxExpression keyExpr;
    if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
        const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand);
        keyExpr = expressionMap.at(&keyVar);
    }
    else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
        keyExpr = buildExpression(model, *actualKeyOperand);
    }
    else {
        throw std::runtime_error("HexalySolver: collection() key must be a variable or constant");
    }

    if (!model.hasCollections()) {
        throw std::runtime_error(
            "HexalySolver: No collection keys provided to model. "
            "Use model.setCollectionLookup(lookup, numberOfCollections)."
        );
    }

    size_t numberOfCollections = model.getNumberOfCollections();

    // Pre-compute results for all collection keys
    std::vector<double> results(numberOfCollections);
    for (size_t i = 0; i < numberOfCollections; i++) {
        double key = static_cast<double>(i);
        auto collectionResult = model.getCollection(key);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(key) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        if (opName == "count") {
            results[i] = static_cast<double>(collection.size());
        }
        else if (opName == "sum") {
            results[i] = std::accumulate(collection.begin(), collection.end(), 0.0);
        }
        else if (opName == "avg") {
            if (collection.empty()) {
                throw std::runtime_error("HexalySolver: avg() is undefined for empty collection at key " +
                                         std::to_string(key));
            }
            results[i] = std::accumulate(collection.begin(), collection.end(), 0.0) / collection.size();
        }
        else if (opName == "max") {
            if (collection.empty()) {
                throw std::runtime_error("HexalySolver: max() is undefined for empty collection at key " +
                                         std::to_string(key));
            }
            results[i] = *std::max_element(collection.begin(), collection.end());
        }
        else if (opName == "min") {
            if (collection.empty()) {
                throw std::runtime_error("HexalySolver: min() is undefined for empty collection at key " +
                                         std::to_string(key));
            }
            results[i] = *std::min_element(collection.begin(), collection.end());
        }
    }

    // Build array and use at() for element selection
    std::vector<hexaly::HxExpression> resultExprs;
    for (double r : results) {
        resultExprs.push_back(hxModel.createConstant(r));
    }

    return hxModel.at(hxModel.array(resultExprs.begin(), resultExprs.end()), round(keyExpr));
}

// Collection membership: element_of(value, Collection(key)) or not_element_of(...)
hexaly::HxExpression HexalySolver::resolveCollectionMembership(
    const Model& model, const Expression& expr, const std::string& opName
) {
    // operands[1] = value
    // operands[2] = collection expression
    const Operand& valueOperand = expr.operands[1];
    const Expression& collectionExpr = std::get<Expression>(expr.operands[2]);

    if (collectionExpr.operands.size() != 1) {
        throw std::runtime_error("HexalySolver: collection() must have exactly 1 argument");
    }

    const Operand& keyOperand = collectionExpr.operands[0];
    bool isNegated = (opName == "not_element_of");

    // Case 1: Constant key and constant value
    if (std::holds_alternative<double>(keyOperand) && std::holds_alternative<double>(valueOperand)) {
        double constantKey = std::get<double>(keyOperand);
        double constantValue = std::get<double>(valueOperand);

        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        bool found = std::find(collection.begin(), collection.end(), constantValue) != collection.end();
        bool result = isNegated ? !found : found;
        return hxModel.createConstant(static_cast<hexaly::hxint>(result ? 1 : 0));
    }

    // Case 2: Constant key, variable value
    if (std::holds_alternative<double>(keyOperand)) {
        double constantKey = std::get<double>(keyOperand);
        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        hexaly::HxExpression valueExpr = buildExpression(model, valueOperand);

        // Build OR of (value == element) for each element
        hexaly::HxExpression result = hxModel.createConstant(static_cast<hexaly::hxint>(0));
        for (double element : collection) {
            result = result || (valueExpr == hxModel.createConstant(element));
        }

        if (isNegated) {
            return !result;
        }
        return result;
    }

    // Unwrap Expression(none, {operand}) to get the actual operand
    const Operand* actualKeyOperand = &keyOperand;
    if (std::holds_alternative<Expression>(keyOperand)) {
        const Expression& keyExprWrapper = std::get<Expression>(keyOperand);
        if (keyExprWrapper._operator == Expression::Operator::none && keyExprWrapper.operands.size() == 1) {
            actualKeyOperand = &keyExprWrapper.operands[0];
        }
    }

    // Case 3: Variable or IndexedVariable key - need to handle all possible collections
    hexaly::HxExpression keyExpr;
    if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
        const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand);
        keyExpr = expressionMap.at(&keyVar);
    }
    else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
        keyExpr = buildExpression(model, *actualKeyOperand);
    }
    else {
        throw std::runtime_error("HexalySolver: collection() key must be a variable or constant");
    }

    if (!model.hasCollections()) {
        throw std::runtime_error("HexalySolver: No collection keys provided to model.");
    }

    size_t numberOfCollections = model.getNumberOfCollections();

    // Build membership expression for each collection, then select based on key
    hexaly::HxExpression valueExpr = buildExpression(model, valueOperand);

    std::vector<hexaly::HxExpression> membershipExprs;
    for (size_t i = 0; i < numberOfCollections; i++) {
        auto collectionResult = model.getCollection(static_cast<double>(i));
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(i) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        // Build OR of (value == element) for this collection
        hexaly::HxExpression membership = hxModel.createConstant(static_cast<hexaly::hxint>(0));
        for (double element : collection) {
            membership = membership || (valueExpr == hxModel.createConstant(element));
        }

        if (isNegated) {
            membership = !membership;
        }

        membershipExprs.push_back(membership);
    }

    return hxModel.at(hxModel.array(membershipExprs.begin(), membershipExprs.end()), round(keyExpr));
}

// Collection item access: at(index, Collection(key))
hexaly::HxExpression HexalySolver::resolveCollectionItem(const Model& model, const Expression& expr) {
    // operands[1] = index
    // operands[2] = collection expression
    const Operand& indexOperand = expr.operands[1];
    const Expression& collectionExpr = std::get<Expression>(expr.operands[2]);

    if (collectionExpr.operands.size() != 1) {
        throw std::runtime_error("HexalySolver: collection() must have exactly 1 argument");
    }

    const Operand& keyOperand = collectionExpr.operands[0];

    // Case 1: Both constant key and constant index
    if (std::holds_alternative<double>(keyOperand) && std::holds_alternative<double>(indexOperand)) {
        double constantKey = std::get<double>(keyOperand);
        size_t index = static_cast<size_t>(std::get<double>(indexOperand));

        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        // CP uses 1-based indexing
        if (index < 1 || index > collection.size()) {
            throw std::runtime_error("HexalySolver: Collection index " + std::to_string(index) +
                                     " out of bounds");
        }

        return hxModel.createConstant(collection[index - 1]);
    }

    // Case 2: Constant key, variable index
    if (std::holds_alternative<double>(keyOperand)) {
        double constantKey = std::get<double>(keyOperand);
        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        hexaly::HxExpression indexExpr = buildExpression(model, indexOperand);

        // Build array and use at() - adjust for 1-based indexing
        std::vector<hexaly::HxExpression> elements;
        for (double element : collection) {
            elements.push_back(hxModel.createConstant(element));
        }

        // CP uses 1-based indexing, so subtract 1
        return hxModel.at(hxModel.array(elements.begin(), elements.end()),
                          round(indexExpr) - hxModel.createConstant(static_cast<hexaly::hxint>(1)));
    }

    // Unwrap Expression(none, {operand}) to get the actual operand
    const Operand* actualKeyOperand = &keyOperand;
    if (std::holds_alternative<Expression>(keyOperand)) {
        const Expression& keyExprWrapper = std::get<Expression>(keyOperand);
        if (keyExprWrapper._operator == Expression::Operator::none && keyExprWrapper.operands.size() == 1) {
            actualKeyOperand = &keyExprWrapper.operands[0];
        }
    }

    // Case 3: Variable or IndexedVariable key
    hexaly::HxExpression keyExpr;
    if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
        const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand);
        keyExpr = expressionMap.at(&keyVar);
    }
    else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
        keyExpr = buildExpression(model, *actualKeyOperand);
    }
    else {
        throw std::runtime_error("HexalySolver: collection() key must be a variable or constant");
    }

    if (!model.hasCollections()) {
        throw std::runtime_error("HexalySolver: No collection keys provided to model.");
    }

    size_t numberOfCollections = model.getNumberOfCollections();
    hexaly::HxExpression indexExpr = buildExpression(model, indexOperand);

    // Build 2D array: for each key, build an array of elements
    // Then use nested at(): at(at(2DArray, key), index-1)
    std::vector<hexaly::HxExpression> collectionArrays;
    for (size_t i = 0; i < numberOfCollections; i++) {
        auto collectionResult = model.getCollection(static_cast<double>(i));
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(i) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        std::vector<hexaly::HxExpression> elements;
        for (double element : collection) {
            elements.push_back(hxModel.createConstant(element));
        }
        collectionArrays.push_back(hxModel.array(elements.begin(), elements.end()));
    }

    // Select collection by key, then element by index (1-based)
    hexaly::HxExpression selectedCollection = hxModel.at(
        hxModel.array(collectionArrays.begin(), collectionArrays.end()),
        round(keyExpr)
    );

    return hxModel.at(selectedCollection,
                      round(indexExpr) - hxModel.createConstant(static_cast<hexaly::hxint>(1)));
}

// Collection access via [] operator: Collection(key)[index]
hexaly::HxExpression HexalySolver::resolveCollectionAccess(const Model& model, const Expression& expr) {
    // operands[0] = collection expression
    // operands[1] = index
    if (!std::holds_alternative<Expression>(expr.operands[0])) {
        throw std::runtime_error("HexalySolver: [] operator requires collection() expression");
    }

    const Expression& collectionExpr = std::get<Expression>(expr.operands[0]);

    if (collectionExpr._operator != Expression::Operator::collection) {
        throw std::runtime_error("HexalySolver: [] operator only supported for collection() expressions");
    }

    if (collectionExpr.operands.size() != 1) {
        throw std::runtime_error("HexalySolver: collection() must have exactly 1 argument");
    }

    const Operand& keyOperand = collectionExpr.operands[0];
    const Operand& indexOperand = expr.operands[1];

    // Case 1: Both constant key and constant index
    if (std::holds_alternative<double>(keyOperand) && std::holds_alternative<double>(indexOperand)) {
        double constantKey = std::get<double>(keyOperand);
        size_t index = static_cast<size_t>(std::get<double>(indexOperand));

        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        if (index < 1 || index > collection.size()) {
            throw std::runtime_error("HexalySolver: Collection index out of bounds");
        }

        return hxModel.createConstant(collection[index - 1]);
    }

    // Case 2: Constant key, variable/constant index
    if (std::holds_alternative<double>(keyOperand)) {
        double constantKey = std::get<double>(keyOperand);
        auto collectionResult = model.getCollection(constantKey);
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(constantKey) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        std::vector<hexaly::HxExpression> elements;
        for (double element : collection) {
            elements.push_back(hxModel.createConstant(element));
        }

        // Handle constant index specially (needs integer for Hexaly at())
        if (std::holds_alternative<double>(indexOperand)) {
            hexaly::hxint index = static_cast<hexaly::hxint>(std::get<double>(indexOperand)) - 1;
            return hxModel.at(hxModel.array(elements.begin(), elements.end()),
                              hxModel.createConstant(index));
        }

        // Variable index - CP uses 1-based indexing
        hexaly::HxExpression indexExpr = buildExpression(model, indexOperand);
        return hxModel.at(hxModel.array(elements.begin(), elements.end()),
                          round(indexExpr) - hxModel.createConstant(static_cast<hexaly::hxint>(1)));
    }

    // Unwrap Expression(none, {operand}) to get the actual operand
    const Operand* actualKeyOperand = &keyOperand;
    if (std::holds_alternative<Expression>(keyOperand)) {
        const Expression& keyExprWrapper = std::get<Expression>(keyOperand);
        if (keyExprWrapper._operator == Expression::Operator::none && keyExprWrapper.operands.size() == 1) {
            actualKeyOperand = &keyExprWrapper.operands[0];
        }
    }

    // Case 3: Variable or IndexedVariable key
    hexaly::HxExpression keyExpr;
    if (std::holds_alternative<std::reference_wrapper<const Variable>>(*actualKeyOperand)) {
        const Variable& keyVar = std::get<std::reference_wrapper<const Variable>>(*actualKeyOperand);
        keyExpr = expressionMap.at(&keyVar);
    }
    else if (std::holds_alternative<IndexedVariable>(*actualKeyOperand)) {
        keyExpr = buildExpression(model, *actualKeyOperand);
    }
    else {
        throw std::runtime_error("HexalySolver: collection() key must be a variable or constant");
    }

    if (!model.hasCollections()) {
        throw std::runtime_error("HexalySolver: No collection keys provided to model.");
    }

    size_t numberOfCollections = model.getNumberOfCollections();

    std::vector<hexaly::HxExpression> collectionArrays;
    for (size_t i = 0; i < numberOfCollections; i++) {
        auto collectionResult = model.getCollection(static_cast<double>(i));
        if (!collectionResult) {
            throw std::runtime_error("HexalySolver: Collection key " + std::to_string(i) +
                                     " not found: " + collectionResult.error());
        }
        const std::vector<double>& collection = collectionResult.value();

        std::vector<hexaly::HxExpression> elements;
        for (double element : collection) {
            elements.push_back(hxModel.createConstant(element));
        }
        collectionArrays.push_back(hxModel.array(elements.begin(), elements.end()));
    }

    hexaly::HxExpression selectedCollection = hxModel.at(
        hxModel.array(collectionArrays.begin(), collectionArrays.end()),
        round(keyExpr)
    );

    // Handle constant index specially (needs integer for Hexaly at())
    if (std::holds_alternative<double>(indexOperand)) {
        hexaly::hxint index = static_cast<hexaly::hxint>(std::get<double>(indexOperand)) - 1;
        return hxModel.at(selectedCollection, hxModel.createConstant(index));
    }

    hexaly::HxExpression indexExpr = buildExpression(model, indexOperand);
    return hxModel.at(selectedCollection,
                      round(indexExpr) - hxModel.createConstant(static_cast<hexaly::hxint>(1)));
}

std::expected<Solution, std::string> HexalySolver::solve(const Model& model) {
    return solve(model, std::numeric_limits<double>::infinity());
}

std::expected<Solution, std::string> HexalySolver::solve(const Model& model, double timeLimit) {
    if (std::isfinite(timeLimit)) {
        optimizer->getParam().setTimeLimit(static_cast<int>(timeLimit));
    }

    try {
        optimizer->solve();
    }
    catch (const hexaly::HxException& e) {
        return std::unexpected(std::string("Hexaly solve failed: ") + e.getMessage());
    }

    hexaly::HxSolution hxSol = optimizer->getSolution();
    hexaly::HxSolutionStatus status = hxSol.getStatus();

    if (status == hexaly::SS_Infeasible) {
        return std::unexpected("Problem is infeasible");
    }

    if (status == hexaly::SS_Inconsistent) {
        return std::unexpected("Model is inconsistent");
    }

    Solution solution(model);

    if (status == hexaly::SS_Optimal) {
        solution.setStatus(Solution::Status::OPTIMAL);
    }
    else if (status == hexaly::SS_Feasible) {
        solution.setStatus(Solution::Status::FEASIBLE);
    }

    // Extract sequence values
    for (const auto& sequence : model.getSequences()) {
        hexaly::HxExpression listVar = sequenceMap.at(&sequence);
        hexaly::HxCollection listVal = hxSol.getCollectionValue(listVar);

        std::vector<int> values;
        for (int i = 0; i < static_cast<int>(sequence.variables.size()); i++) {
            // List contains 0-based values, convert to 1-based
            values.push_back(static_cast<int>(listVal.get(i)) + 1);
        }
        solution.setSequenceValues(sequence, values);
    }

    // Extract variable values (including deduced variables)
    for (const Variable& var : model.getAllVariables()) {
        hexaly::HxExpression hxExpr = expressionMap.at(&var);
        double value;

        // Check Hexaly expression type (may differ from CP variable type for deduced vars)
        if (hxExpr.isInt()) {
            value = static_cast<double>(hxSol.getIntValue(hxExpr));
        }
        else {
            value = hxSol.getDoubleValue(hxExpr);
        }

        solution.setVariableValue(var, value);
    }

    return solution;
}

} // namespace CP
