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

// Define built-in functions
template <>
void LIMEX::Handle<CP::Expression,CP::Expression>::initialize() {
  add(
    std::string("if_then_else"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 3) throw std::runtime_error("LIMEX: if_then_else() requires exactly two arguments");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("if_then_else"), args[0], args[1], args[2] } );
    }
  );

  add(
    std::string("n_ary_if"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: n_ary_if() requires at least one argument");
      std::vector<CP::Operand> operands = { CP::Expression::getCustomIndex("n_ary_if") };
      operands.insert(operands.end(), args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::custom, std::move(operands));
    }
  );

  add(
    std::string("abs"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: abs() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("if_then_else"), args[0] >= 0, args[0], -args[0] } );
    }
  );

  add(
    std::string("pow"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 2) throw std::runtime_error("LIMEX: pow() requires exactly two arguments");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("pow"), args[0], args[1] });
    }
  );

  add(
    std::string("sqrt"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: sqrt() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("pow"), args[0], 1.0/2 });
    }
  );

  add(
    std::string("cbrt"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() != 1) throw std::runtime_error("LIMEX: cbrt() requires exactly one argument");
      return CP::Expression( CP::Expression::Operator::custom, { CP::Expression::getCustomIndex("pow"), args[0], 1.0/3 });
    }
  );

  add(
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

  add(
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

  add(
    std::string("count"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      return args.size();
    }
  );
  
  add(
    std::string("min"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: min{} requires at least one argument");
      std::vector<CP::Operand> operands = { CP::Expression::getCustomIndex("min") };
      operands.insert(operands.end(), args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::custom, std::move(operands));
    }
  );

  add(
    std::string("max"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.empty()) throw std::runtime_error("LIMEX: max{} requires at least one argument");
      std::vector<CP::Operand> operands = { CP::Expression::getCustomIndex("max") };
      operands.insert(operands.end(), args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::custom, std::move(operands));
    }
  );

  add(
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

  add(
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

  add(
    std::string("at"), 
    [](const std::vector<CP::Expression>& args) -> CP::Expression
    {
      if (args.size() < 2) throw std::runtime_error("LIMEX: at() requires at least two arguments");
      std::vector<CP::Operand> operands = { CP::Expression::getCustomIndex("at") };
      operands.insert(operands.end(), args.begin(), args.end());
      return CP::Expression(CP::Expression::Operator::custom, std::move(operands));
    }
  );
}

