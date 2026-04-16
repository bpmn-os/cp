#pragma once

#include <limex.h>
#include "cp.h"

/**************************************************
 ** LIMEX::Handle<CP::Expression,CP::Expression>
 **************************************************/

template <>
inline CP::Expression LIMEX::Handle<CP::Expression,CP::Expression>::indexedEvaluation( const CP::Expression& collection, const CP::Expression& index ) const {
  CP::Expression container(CP::Expression::Operator::collection, { collection });
  return CP::Expression(CP::Expression::Operator::at, { container, index });
}

template <>
inline CP::Expression LIMEX::Handle<CP::Expression,CP::Expression>::aggregateEvaluation( const std::string& name, const CP::Expression& collection ) const {
  CP::Expression container(CP::Expression::Operator::collection, { collection });
  return CP::Expression(CP::Expression::Operator::custom, { CP::Expression::getCustomIndex(name), container});
}

template <>
inline CP::Expression LIMEX::Handle<CP::Expression,CP::Expression>::filteredAggregateEvaluation(
  const std::string& name,
  const CP::Expression& collection,
  const LIMEX::Node<CP::Expression,CP::Expression>& conditions,
  const std::vector<CP::Expression>& variableValues,
  const std::vector<CP::Expression>& collectionValues
) const {
  // Create iterator placeholder using Operator::custom with "_iterator" name
  CP::Expression iteratorPlaceholder(CP::Expression::Operator::custom,
    { CP::Expression::getCustomIndex("_iterator") });

  // Evaluate conditions with iterator placeholder
  std::vector<CP::Expression> localVars = variableValues;
  localVars.push_back(iteratorPlaceholder);
  CP::Expression conditionExpr = conditions.evaluate(localVars, collectionValues);

  // Build filtered aggregate expression (3 operands = filtered)
  CP::Expression container(CP::Expression::Operator::collection, { collection });
  return CP::Expression(CP::Expression::Operator::custom, {
    CP::Expression::getCustomIndex(name),
    container,
    conditionExpr
  });
}

template <>
inline CP::Expression LIMEX::Handle<CP::Expression,CP::Expression>::membershipEvaluation(
  const std::string& name,
  const CP::Expression& element,
  const CP::Expression& collection
) const {
  CP::Expression container(CP::Expression::Operator::collection, { collection });
  return CP::Expression(CP::Expression::Operator::custom, {
    CP::Expression::getCustomIndex(name),
    element,
    container
  });
}

// Define built-in functions and aggregators
template <>
void LIMEX::Handle<CP::Expression,CP::Expression>::initialize() {
  // Functions
  addFunction(
    std::string("if_then_else"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 3) throw std::runtime_error("LIMEX: if_then_else() requires exactly three arguments");
      return CP::Expression( CP::Expression::Operator::if_then_else, { args[0], args[1], args[2] } );
    }
  );

  addFunction(
    std::string("n_ary_if"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: n_ary_if() requires at least one argument");
      std::vector<CP::Operand> operands(args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::n_ary_if, std::move(operands));
    }
  );

  addFunction(
    std::string("abs"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: abs() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::if_then_else, { args[0] >= 0, args[0], -args[0] } );
    }
  );

  addFunction(
    std::string("pow"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 2) throw std::runtime_error("LIMEX: pow() requires exactly two arguments");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("pow"), args[0], args[1] });
    }
  );

  addFunction(
    std::string("sqrt"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: sqrt() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("pow"), args[0], 1.0/2 });
    }
  );

  addFunction(
    std::string("cbrt"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: cbrt() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("pow"), args[0], 1.0/3 });
    }
  );

  addFunction(
    std::string("log"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: log() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("log"), args[0] });
    }
  );

  addFunction(
    std::string("exp"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: exp() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("exp"), args[0] });
    }
  );

  // Aggregators
  addAggregator(
    std::string("sum"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      CP::Expression result(0.0);
      for ( CP::Expression value : args ) {
        result = result + value;
      }
      return result;
    }
  );

  addAggregator(
    std::string("avg"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: avg{} requires at least one argument");
      CP::Expression result(0.0);
      for ( CP::Expression value : args ) {
        result = result + value;
      }
      return result / args.size();
    }
  );

  addAggregator(
    std::string("count"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      return args.size();
    }
  );

  addAggregator(
    std::string("min"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: min{} requires at least one argument");
      std::vector<CP::Operand> operands = { CP::Expression::getCustomIndex("min") };
      operands.insert(operands.end(), args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::custom, std::move(operands));
    }
  );

  addAggregator(
    std::string("max"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: max{} requires at least one argument");
      std::vector<CP::Operand> operands = { CP::Expression::getCustomIndex("max") };
      operands.insert(operands.end(), args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::custom, std::move(operands));
    }
  );

  addAggregator(
    std::string("element_of"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: ∈ {} requires at least one argument");
      CP::Cases cases;
      for (size_t i = 1; i < args.size(); ++i) {
        cases.push_back( { args[0] == args[i], true } );
      }
      return CP::n_ary_if( std::move(cases), false );
    }
  );

  addAggregator(
    std::string("not_element_of"),
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: ∉ {} requires at least one argument");
      CP::Cases cases;
      for (size_t i = 1; i < args.size(); ++i) {
        cases.push_back( { args[0] == args[i], false } );
      }
      return CP::n_ary_if( std::move(cases), true );
    }
  );
}

