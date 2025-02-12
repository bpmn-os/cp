 /**
 ******************************************************************************
 *
 *  Constraint programming interface
 *
 ******************************************************************************
 */

#pragma once

#include <memory>
#include <deque>
#include <vector>
#include <limits>
#include <string>
#include <format>
#include <ranges>
#include <variant>
#include <optional>
#include <unordered_map>
#include <functional>
#include <cmath>

namespace CP {

struct Expression;

/*******************************************
 * Variable
 ******************************************/

/**
 * @brief Represents a variable in a constraint program.
 */
struct Variable {
  /**
   * @brief Enum class representing the type of the variable.
   */
  enum class Type { BOOLEAN, INTEGER, REAL };

  /**
   * @brief Constructs an unbounded variable with given type.
   * 
   * @param type The type of the variable (BOOLEAN, INTEGER, REAL).
   */
  inline Variable(Type type, std::string name ) 
    : type(type)
    , name(std::move(name))
    , lowerBound( type == Type::BOOLEAN ? 0 : std::numeric_limits<double>::lowest() )
    , upperBound( type == Type::BOOLEAN ? 1 : std::numeric_limits<double>::max() )
    , deducedFrom(nullptr)
  {
  };
  
  /**
   * @brief Constructs a variable with given type and bounds.
   * 
   * @param type The type of the variable (BOOLEAN, INTEGER, REAL).
   * @param lowerBound The lower bound of the variable.
   * @param upperBound The upper bound of the variable.
   */
  inline Variable(Type type, std::string name, double lowerBound, double upperBound ) 
    : type(type)
    , name(std::move(name))
    , lowerBound(lowerBound)
    , upperBound(upperBound)
    , deducedFrom(nullptr)
  {
  };
  
  /**
   * @brief Constructs a variable which is deduced from an expression.
   * 
   * @tparam ExpressionType The type of the expression used to initialize the variable.
   * @param type The type of the variable (BOOLEAN, INTEGER, REAL).
   * @param expression The expression used to initialize the variable.
   */
  inline Variable(Type type, std::string name, const Expression& expression ) 
    : type(type)
    , name(std::move(name))
    , lowerBound( type == Type::BOOLEAN ? 0 : std::numeric_limits<double>::lowest() )
    , upperBound( type == Type::BOOLEAN ? 1 : std::numeric_limits<double>::max() )
    , deducedFrom( std::make_unique<Expression>(expression) )
  {
  }

  /**
   * @brief Constructs a variable which is deduced from another variable.
   * 
   * @param type The type of the variable (BOOLEAN, INTEGER, REAL).
   * @param other The other variable used to deduce the variable.
   */
  Variable(Type type, std::string name, const Variable& other );

  Variable(Variable&&) noexcept = default; // Define move constructor
//  Variable& operator=(Variable&&) noexcept = default; // Define move assignment
  Variable(const Variable&) = delete; // Disable copy constructor
  Variable& operator=(const Variable&) = delete; // Disable copy assignment
    
  Type type;
  std::string name;
  double lowerBound;
  double upperBound;
  std::unique_ptr<Expression> deducedFrom; ///< Pointer to an expression the variable is deduced from, or nullptr.
  
  inline Expression operator-() const;
  inline Expression operator!() const;

  inline Expression operator&&(double constant) const;
  inline Expression operator||(double constant) const;
  inline Expression operator+(double constant) const;
  inline Expression operator-(double constant) const;
  inline Expression operator*(double constant) const;
  inline Expression operator/(double constant) const;
  inline Expression operator<(double constant) const;
  inline Expression operator>(double constant) const;
  inline Expression operator<=(double constant) const;
  inline Expression operator>=(double constant) const;
  inline Expression operator==(double constant) const;
  inline Expression operator!=(double constant) const;

  inline Expression operator&&(const Variable& variable) const;
  inline Expression operator||(const Variable& variable) const;
  inline Expression operator+(const Variable& variable) const;
  inline Expression operator-(const Variable& variable) const;
  inline Expression operator*(const Variable& variable) const;
  inline Expression operator/(const Variable& variable) const;
  inline Expression operator<(const Variable& variable) const;
  inline Expression operator>(const Variable& variable) const;
  inline Expression operator<=(const Variable& variable) const;
  inline Expression operator>=(const Variable& variable) const;
  inline Expression operator==(const Variable& variable) const;
  inline Expression operator!=(const Variable& variable) const;

  inline Expression operator&&(const Expression& expression) const;
  inline Expression operator||(const Expression& expression) const;
  inline Expression operator+(const Expression& expression) const;
  inline Expression operator-(const Expression& expression) const;
  inline Expression operator*(const Expression& expression) const;
  inline Expression operator/(const Expression& expression) const;
  inline Expression operator<(const Expression& expression) const;
  inline Expression operator>(const Expression& expression) const;
  inline Expression operator<=(const Expression& expression) const;
  inline Expression operator>=(const Expression& expression) const;
  inline Expression operator==(const Expression& expression) const;
  inline Expression operator!=(const Expression& expression) const;

  inline Expression implies(const Expression& expression) const;

  inline std::string stringify() const;

};

template<typename T>
class reference_vector : public std::vector<std::reference_wrapper<T>> {
public:
  // Overloading the [] operator to return a const reference to T
  inline const T& operator[](std::size_t index) const {
    return std::vector<std::reference_wrapper<T>>::at(index).get();
  }

  // Overloading the [] operator to return a reference to T
  inline T& operator[](std::size_t index) {
    return std::vector<std::reference_wrapper<T>>::at(index).get();
  }
};

/*******************************************
 * IndexedVariable(s)
 ******************************************/

struct IndexedVariables;

struct IndexedVariable {
  inline IndexedVariable(const IndexedVariables& container, const Variable& index) : container(container), index(index) {}
  const IndexedVariables& container;
  const Variable& index;
  inline std::string stringify() const;
};

struct IndexedVariables {
  Variable::Type type;
  std::string name;
  inline IndexedVariables(Variable::Type type, std::string name) : type(type), name(std::move(name)) {} 
  IndexedVariables(const IndexedVariables&) = delete; // Disable copy constructor
  IndexedVariables& operator=(const IndexedVariables&) = delete; // Disable copy assignment

  inline const Variable& operator[](std::size_t index) const {
    return _references.at(index);
  }

  inline IndexedVariable operator[](const Variable& index) const {
     return IndexedVariable(*this,index);
  }

  template <typename... Args>
  inline void emplace_back(Args&&... args) {
    _variables.emplace_back(type, name + "[" + std::to_string(_variables.size()) + "]", std::forward<Args>(args)... );
    _references.emplace_back(_variables.back());
  }

  inline size_t size() const { return _variables.size(); }
  inline bool empty() const { return _variables.empty(); }
  inline auto begin() { return _variables.begin(); }
  inline auto begin() const { return _variables.cbegin(); }
  inline auto end() { return _variables.end(); }
  inline auto end() const { return _variables.cend(); }
    
  inline std::string stringify() const {
    std::string result = name + " := {";
    for ( const Variable& variable : _variables ) {
      result += " " + variable.stringify() + ",";
    }
    if (!empty()) {
      result.back() = ' ';
    }
    result += "}";
    return result;
  }
private:
  std::deque<Variable> _variables;
  reference_vector<Variable> _references;
};

inline std::string IndexedVariable::stringify() const { return container.name + "[" + index.name + "]"; }

/*******************************************
 * Expression
 ******************************************/
using Operand = std::variant< size_t, double, std::reference_wrapper<const Variable>, Expression>;

/**
 * @brief Represents an expression.
 */
struct Expression {
  enum class Operator {
    none,
    negate,
    logical_not,
    logical_and,
    logical_or,
    add,
    subtract,
    multiply,
    divide,
    custom,
    less_than,
    less_or_equal,
    greater_than,
    greater_or_equal,
    equal,
    not_equal
  };
  inline Expression() : Expression(Operator::none,{0.0}) {};
  inline Expression(double constant) : _operator(Operator::none), operands({constant}) {};
  inline Expression(const Variable& variable) : _operator(Operator::none), operands({std::ref(variable)}) {};
  inline Expression(Operator _operator, const std::vector< Operand >& operands) : _operator(_operator), operands(operands) {};

  inline Expression operator-() const { return Expression(Operator::negate, {*this}); };
  inline Expression operator!() const { return Expression(Operator::logical_not, {*this}); };

  inline Expression operator&&(double constant) const { return Expression(Expression::Operator::logical_and, {(*this),constant});}
  inline Expression operator||(double constant) const { return Expression(Expression::Operator::logical_or, {(*this),constant});}
  inline Expression operator+(double constant) const { return Expression(Expression::Operator::add, {(*this),constant});}
  inline Expression operator-(double constant) const { return Expression(Expression::Operator::subtract, {(*this),constant});}
  inline Expression operator*(double constant) const { return Expression(Expression::Operator::multiply, {(*this),constant});}
  inline Expression operator/(double constant) const { return Expression(Expression::Operator::divide, {(*this),constant});}
  inline Expression operator<(double constant) const  { return Expression(Operator::less_than, {*this,constant}); };
  inline Expression operator>(double constant) const  { return Expression(Operator::greater_than, {*this,constant}); };
  inline Expression operator<=(double constant) const { return Expression(Operator::less_or_equal, {*this,constant}); };
  inline Expression operator>=(double constant) const { return Expression(Operator::greater_or_equal, {*this,constant}); };
  inline Expression operator==(double constant) const { return Expression(Operator::equal, {*this,constant}); };
  inline Expression operator!=(double constant) const { return Expression(Operator::not_equal, {*this,constant}); };

  inline Expression operator&&(const Variable& variable) const { return Expression(Expression::Operator::logical_and, {(*this),std::ref(variable)});}
  inline Expression operator||(const Variable& variable) const { return Expression(Expression::Operator::logical_or, {(*this),std::ref(variable)});}
  inline Expression operator+(const Variable& variable) const { return Expression(Expression::Operator::add, {(*this),std::ref(variable)});}
  inline Expression operator-(const Variable& variable) const { return Expression(Expression::Operator::subtract, {(*this),std::ref(variable)});}
  inline Expression operator*(const Variable& variable) const { return Expression(Expression::Operator::multiply, {(*this),std::ref(variable)});}
  inline Expression operator/(const Variable& variable) const { return Expression(Expression::Operator::divide, {(*this),std::ref(variable)});}
  inline Expression operator<(const Variable& variable) const  { return Expression(Operator::less_than, {*this,std::ref(variable)}); };
  inline Expression operator>(const Variable& variable) const  { return Expression(Operator::greater_than, {*this,std::ref(variable)}); };
  inline Expression operator<=(const Variable& variable) const { return Expression(Operator::less_or_equal, {*this,std::ref(variable)}); };
  inline Expression operator>=(const Variable& variable) const { return Expression(Operator::greater_or_equal, {*this,std::ref(variable)}); };
  inline Expression operator==(const Variable& variable) const { return Expression(Operator::equal, {*this,std::ref(variable)}); };
  inline Expression operator!=(const Variable& variable) const { return Expression(Operator::not_equal, {*this,std::ref(variable)}); };

  inline Expression operator&&(const Expression& expression) const { return Expression(Operator::logical_and, {*this,expression}); };
  inline Expression operator||(const Expression& expression) const { return Expression(Operator::logical_or, {*this,expression}); };
  inline Expression operator+(const Expression& expression) const  { return Expression(Operator::add, {*this,expression}); };
  inline Expression operator-(const Expression& expression) const  { return Expression(Operator::subtract, {*this,expression}); };
  inline Expression operator*(const Expression& expression) const  { return Expression(Operator::multiply, {*this,expression}); };
  inline Expression operator/(const Expression& expression) const  { return Expression(Operator::divide, {*this,expression}); };
  inline Expression operator<(const Expression& expression) const  { return Expression(Operator::less_than, {*this,expression}); };
  inline Expression operator>(const Expression& expression) const  { return Expression(Operator::greater_than, {*this,expression}); };
  inline Expression operator<=(const Expression& expression) const { return Expression(Operator::less_or_equal, {*this,expression}); };
  inline Expression operator>=(const Expression& expression) const { return Expression(Operator::greater_or_equal, {*this,expression}); };
  inline Expression operator==(const Expression& expression) const { return Expression(Operator::equal, {*this,expression}); };
  inline Expression operator!=(const Expression& expression) const { return Expression(Operator::not_equal, {*this,expression}); };

  inline Expression implies(const Expression& expression) const { return !(*this) || expression; };

  inline static std::string stringify(const Operand& term, bool parenthesize = true) {
    std::string result;
    if (std::holds_alternative<double>(term)) {
      auto constant = std::get<double>(term);
      result += std::format("{:.2f}", constant);
    }
    else if (std::holds_alternative<std::reference_wrapper<const CP::Variable>>(term)) {
      auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(term).get();
      result += variable.name;
    }
    else if ( std::holds_alternative<Expression>(term) ) {
      auto& expression = std::get<Expression>(term);
      if ( expression._operator != Operator::custom && parenthesize ) {
        result += "( " + expression.stringify() + " )";
      }
      else {
        result += expression.stringify();
      }
    }
    else {
      throw std::logic_error("CP: unexpected operand");
    }
    return result;
  }
  
  inline static std::string stringify(const std::string& op, const Operand& term) {
    return op + stringify(term);
  }


  inline static std::string stringify(const Operand& lhs, const std::string& op, const Operand& rhs) {
    bool parenthesize = ( op != "<" && op != ">" && op != "<=" && op != ">=" && op != "==" && op != "!="); 
    return stringify(lhs, parenthesize) + " " + op + " " + stringify(rhs, parenthesize);
  };

  inline std::string stringify() const {
    using enum Operator;
    switch (_operator) {
      case none:
      {
        return stringify(operands[0]);
      }
      case negate:
      {
        return stringify("-", operands[0]);
      }
      case logical_not:
      {
        return stringify("!", operands[0]);
      }
      case logical_and:
      {
        return stringify(operands[0], "&&", operands[1]);
      }
      case logical_or:
      {
        return stringify(operands[0], "||", operands[1]);
      }
      case add:
      {
        return stringify(operands[0], "+", operands[1]);
      }
      case subtract:
      {
        return stringify(operands[0], "-", operands[1]);
      }
      case multiply:
      {
        return stringify(operands[0], "*", operands[1]);
      }
      case divide:
      {
        return stringify(operands[0], "/", operands[1]);
      }
      case custom:
      {
        auto index = std::get<size_t>(operands.front());
        std::string result = customOperators[index] + "( ";
        for ( size_t i = 1; i < operands.size(); i++) {
          result += stringify(operands[i],false) + ", ";
        }
        result.pop_back();
        result.back() = ' ';
        result += ")";
        return result;
      }
      case less_than:
      {
        return stringify(operands[0], "<", operands[1]);
      }
      case less_or_equal:
      {
        return stringify(operands[0], "<=", operands[1]);
      }
      case greater_than:
      {
        return stringify(operands[0], ">", operands[1]);
      }
      case greater_or_equal:
      {
        return stringify(operands[0], ">=", operands[1]);
      }
      case equal:
      {
        return stringify(operands[0], "==", operands[1]);
      }
      case not_equal:
      {
        return stringify(operands[0], "!=", operands[1]);
      }
      default:
      {
        throw std::logic_error("CP: unexpected operator");
      }
    }
  };
  
  Operator _operator;
  std::vector< Operand > operands;
  inline static std::vector<std::string> customOperators = {};
  inline static size_t getCustomIndex(std::string name) {
    for ( size_t i = 0; i < customOperators.size(); i++) {
      if ( customOperators[i] == name ) {
        return i;
      }
    }
    customOperators.push_back(name);
    return customOperators.size()-1;
  } 
};

inline std::optional<std::pair<Expression, Expression>> isImplication( const Expression& expression ) {
  if (
    expression._operator == Expression::Operator::logical_or &&
    expression.operands.size() == 2 &&
    std::holds_alternative<Expression>(expression.operands.front()) &&
    std::holds_alternative<Expression>(expression.operands.back()) &&
    std::get<Expression>(expression.operands.front())._operator  == Expression::Operator::logical_not
  ) {
    auto& negated_condition = std::get<Expression>(expression.operands.front());
    if ( std::holds_alternative<double>(negated_condition.operands.front()) ) {
      return std::nullopt;
    }
    auto condition =
      std::holds_alternative<Expression>(negated_condition.operands.front()) ? 
      std::get<Expression>(negated_condition.operands.front()) : 
      Expression(std::get<std::reference_wrapper<const CP::Variable>>(negated_condition.operands.front()).get())
    ; 
    return std::make_pair(condition, std::get<Expression>(expression.operands.back()));
  }
  return std::nullopt;
};

/*******************************************
 * Variable (implementation)
 ******************************************/

inline Variable::Variable(Type type, std::string name, const Variable& other ) 
  : type(type)
  , name(std::move(name))
  , lowerBound( type == Type::BOOLEAN ? 0 : std::numeric_limits<double>::lowest() )
  , upperBound( type == Type::BOOLEAN ? 1 : std::numeric_limits<double>::max() )
  , deducedFrom( std::make_unique<Expression>(other) )
//  , deducedFrom( std::make_unique<Expression>(Expression::Operator::none,{std::ref(other)}) )
{
};

inline Expression Variable::operator-() const { return Expression(Expression::Operator::negate, {std::ref(*this)});}
inline Expression Variable::operator!() const { return Expression(Expression::Operator::logical_not, {std::ref(*this)});}

inline Expression Variable::operator&&(double constant) const { return Expression(Expression::Operator::logical_and, {std::ref(*this),constant});}
inline Expression Variable::operator||(double constant) const { return Expression(Expression::Operator::logical_or, {std::ref(*this),constant});}
inline Expression Variable::operator+(double constant) const { return Expression(Expression::Operator::add, {std::ref(*this),constant});}
inline Expression Variable::operator-(double constant) const { return Expression(Expression::Operator::subtract, {std::ref(*this),constant});}
inline Expression Variable::operator*(double constant) const { return Expression(Expression::Operator::multiply, {std::ref(*this),constant});}
inline Expression Variable::operator/(double constant) const { return Expression(Expression::Operator::divide, {std::ref(*this),constant});}
inline Expression Variable::operator<(double constant) const  { return Expression(Expression::Operator::less_than, {std::ref(*this),constant}); };
inline Expression Variable::operator>(double constant) const  { return Expression(Expression::Operator::greater_than, {std::ref(*this),constant}); };
inline Expression Variable::operator<=(double constant) const { return Expression(Expression::Operator::less_or_equal, {std::ref(*this),constant}); };
inline Expression Variable::operator>=(double constant) const { return Expression(Expression::Operator::greater_or_equal, {std::ref(*this),constant}); };
inline Expression Variable::operator==(double constant) const { return Expression(Expression::Operator::equal, {std::ref(*this),constant}); };
inline Expression Variable::operator!=(double constant) const { return Expression(Expression::Operator::not_equal, {std::ref(*this),constant}); };

inline Expression Variable::operator&&(const Variable& variable) const { return Expression(Expression::Operator::logical_and, {std::ref(*this),std::ref(variable)});}
inline Expression Variable::operator||(const Variable& variable) const { return Expression(Expression::Operator::logical_or, {std::ref(*this),std::ref(variable)});}
inline Expression Variable::operator+(const Variable& variable) const { return Expression(Expression::Operator::add, {std::ref(*this),std::ref(variable)});}
inline Expression Variable::operator-(const Variable& variable) const { return Expression(Expression::Operator::subtract, {std::ref(*this),std::ref(variable)});}
inline Expression Variable::operator*(const Variable& variable) const { return Expression(Expression::Operator::multiply, {std::ref(*this),std::ref(variable)});}
inline Expression Variable::operator/(const Variable& variable) const { return Expression(Expression::Operator::divide, {std::ref(*this),std::ref(variable)});}
inline Expression Variable::operator<(const Variable& variable) const  { return Expression(Expression::Operator::less_than, {std::ref(*this),std::ref(variable)}); };
inline Expression Variable::operator>(const Variable& variable) const  { return Expression(Expression::Operator::greater_than, {std::ref(*this),std::ref(variable)}); };
inline Expression Variable::operator<=(const Variable& variable) const { return Expression(Expression::Operator::less_or_equal, {std::ref(*this),std::ref(variable)}); };
inline Expression Variable::operator>=(const Variable& variable) const { return Expression(Expression::Operator::greater_or_equal, {std::ref(*this),std::ref(variable)}); };
inline Expression Variable::operator==(const Variable& variable) const { return Expression(Expression::Operator::equal, {std::ref(*this),std::ref(variable)}); };
inline Expression Variable::operator!=(const Variable& variable) const { return Expression(Expression::Operator::not_equal, {std::ref(*this),std::ref(variable)}); };

inline Expression Variable::operator&&(const Expression& expression) const { return Expression(Expression::Operator::logical_and, {std::ref(*this),expression});}
inline Expression Variable::operator||(const Expression& expression) const { return Expression(Expression::Operator::logical_or, {std::ref(*this),expression});}
inline Expression Variable::operator+(const Expression& expression) const { return Expression(Expression::Operator::add, {std::ref(*this),expression});}
inline Expression Variable::operator-(const Expression& expression) const { return Expression(Expression::Operator::subtract, {std::ref(*this),expression});}
inline Expression Variable::operator*(const Expression& expression) const { return Expression(Expression::Operator::multiply, {std::ref(*this),expression});}
inline Expression Variable::operator/(const Expression& expression) const { return Expression(Expression::Operator::divide, {std::ref(*this),expression});}
inline Expression Variable::operator<(const Expression& expression) const { return Expression(Expression::Operator::less_than, {std::ref(*this),expression});}
inline Expression Variable::operator>(const Expression& expression) const { return Expression(Expression::Operator::greater_than, {std::ref(*this),expression});}
inline Expression Variable::operator<=(const Expression& expression) const { return Expression(Expression::Operator::less_or_equal, {std::ref(*this),expression});}
inline Expression Variable::operator>=(const Expression& expression) const { return Expression(Expression::Operator::greater_or_equal, {std::ref(*this),expression});}
inline Expression Variable::operator==(const Expression& expression) const { return Expression(Expression::Operator::equal, {std::ref(*this),expression});}
inline Expression Variable::operator!=(const Expression& expression) const { return Expression(Expression::Operator::not_equal, {std::ref(*this),expression});}

inline Expression Variable::implies(const Expression& expression) const { return !(*this) || expression; };

inline std::string Variable::stringify() const {
  if ( deducedFrom ) {
    return name + " := " + deducedFrom->stringify();
  }
    
  if ( type == Type::BOOLEAN ) {
    if ( lowerBound == upperBound ) {
      return name + " := " + ( lowerBound ? "true" : "false" );
    }
    return name + " ∈ { false, true }"; 
  }
  else if ( type == Type::INTEGER ) {
    if ( lowerBound == upperBound ) {
      return name + " := " + std::to_string( (int)lowerBound );
    }
    return name + " ∈ { " + ( lowerBound == std::numeric_limits<double>::lowest() ? std::string("-infinity") : std::to_string( (int)lowerBound ) ) + ", ..., " + ( upperBound == std::numeric_limits<double>::max() ? std::string("infinity") : std::to_string( (int)upperBound ) ) + " }";
  }

  if ( lowerBound == upperBound ) {
    return name + " := " + std::format("{:.2f}", lowerBound);
  }     
  return name + " ∈ [ " + ( lowerBound == std::numeric_limits<double>::lowest() ? "-infinity" : std::format("{:.2f}", lowerBound) ) + ", " + ( upperBound == std::numeric_limits<double>::max() ? "infinity" : std::format("{:.2f}", upperBound) ) + " ]";
}

/*******************************************
 * Left side operators
 ******************************************/

inline Expression operator&&(bool constant, const Variable& variable) { return Expression(Expression::Operator::logical_and, {(double)constant,std::ref(variable)}); };
inline Expression operator||(bool constant, const Variable& variable) { return Expression(Expression::Operator::logical_or, {(double)constant,std::ref(variable)}); };
inline Expression operator+(double constant, const Variable& variable) { return Expression(Expression::Operator::add, {constant,std::ref(variable)}); };
inline Expression operator-(double constant, const Variable& variable) { return Expression(Expression::Operator::subtract, {constant,std::ref(variable)}); };
inline Expression operator*(double constant, const Variable& variable) { return Expression(Expression::Operator::multiply, {constant,std::ref(variable)}); };
inline Expression operator/(double constant, const Variable& variable) { return Expression(Expression::Operator::divide, {constant,std::ref(variable)}); };

inline Expression operator&&(bool constant, const Expression& expression) { return Expression(Expression::Operator::logical_and, {(double)constant,expression}); };
inline Expression operator||(bool constant, const Expression& expression) { return Expression(Expression::Operator::logical_or, {(double)constant,expression}); };
inline Expression operator+(double constant, const Expression& expression) { return Expression(Expression::Operator::add, {constant,expression}); };
inline Expression operator-(double constant, const Expression& expression) { return Expression(Expression::Operator::subtract, {constant,expression}); };
inline Expression operator*(double constant, const Expression& expression) { return Expression(Expression::Operator::multiply, {constant,expression}); };
inline Expression operator/(double constant, const Expression& expression) { return Expression(Expression::Operator::divide, {constant,expression}); };



/*******************************************
 * Yoda comparisons
 ******************************************/

inline Expression operator<(double constant, const Variable& variable) {  return Expression(Expression::Operator::less_than, {constant,std::ref(variable)}); };
inline Expression operator>(double constant, const Variable& variable) {  return Expression(Expression::Operator::greater_than, {constant,std::ref(variable)}); };
inline Expression operator<=(double constant, const Variable& variable) {  return Expression(Expression::Operator::less_or_equal, {constant,std::ref(variable)}); };
inline Expression operator>=(double constant, const Variable& variable) {  return Expression(Expression::Operator::greater_or_equal, {constant,std::ref(variable)}); };
inline Expression operator==(double constant, const Variable& variable) {  return Expression(Expression::Operator::equal, {constant,std::ref(variable)}); };
inline Expression operator!=(double constant, const Variable& variable) {  return Expression(Expression::Operator::not_equal, {constant,std::ref(variable)}); };

inline Expression operator<(double constant, const Expression& expression) {  return Expression(Expression::Operator::less_than, {constant,expression}); };
inline Expression operator>(double constant, const Expression& expression) {  return Expression(Expression::Operator::greater_than, {constant,expression}); };
inline Expression operator<=(double constant, const Expression& expression) {  return Expression(Expression::Operator::less_or_equal, {constant,expression}); };
inline Expression operator>=(double constant, const Expression& expression) {  return Expression(Expression::Operator::greater_or_equal, {constant,expression}); };
inline Expression operator==(double constant, const Expression& expression) {  return Expression(Expression::Operator::equal, {constant,expression}); };
inline Expression operator!=(double constant, const Expression& expression) {  return Expression(Expression::Operator::not_equal, {constant,expression}); };

/*******************************************
 * Sequence
 ******************************************/

/**
 * @brief Represents a collection of integer variables with the property that the variable values are a permutation of {1, ..., n}.
 */
struct Sequence {
  inline Sequence(std::string name, size_t n) {
    for ( size_t i = 0; i < n; i++ ) {
      _variables.emplace_back(Variable::Type::INTEGER, name + '[' + std::to_string(i) + ']', 1, n );
      variables.push_back( _variables.back() );
    }
  };
  Sequence(const Sequence&) = delete; // Disable copy constructor
  Sequence& operator=(const Sequence&) = delete; // Disable copy assignment
  reference_vector<const Variable> variables;

  inline std::string stringify() const {
    std::string result = "(";
    for ( const Variable& variable : variables ) {
      result += " " + variable.name + ",";
    }
    result.back() = ' ';
    result += ") is permutation of { 1, " + std::string( variables.size() > 2 ? "..., " : "" ) + std::to_string(variables.size()) + " }";
    return result;
  };
private:
  std::deque<Variable> _variables;
};

/*******************************************
 * Custom operators
 ******************************************/

template<typename... Terms>
Expression customOperator(const std::string& name, Terms&&... terms) {
  // Static assert to ensure all terms are either arithmetic, Variable, or Expression
  static_assert(((
    std::is_arithmetic_v<std::decay_t<Terms>> || 
    std::is_same_v<std::decay_t<Terms>, Variable> ||
    std::is_same_v<std::decay_t<Terms>, Expression>) && ...),
    "CP: All terms must be a number, variable, or expression"
  );

  std::vector< Operand > operands;

  operands.push_back( Expression::getCustomIndex(name) );

  // Lambda to handle each term and push it into operands
  auto add_term = [&operands](auto&& term) {
    if constexpr (std::is_arithmetic_v<std::decay_t<decltype(term)>>) {
      operands.push_back((double)term);
    }
    else if constexpr (std::is_same_v<std::decay_t<decltype(term)>, CP::Variable>) {
      operands.push_back(std::ref(term));
    }
    else if constexpr (std::is_same_v<std::decay_t<decltype(term)>, Expression>) {
      operands.push_back(std::move(term));
    }
  };

  // Expand the parameter pack and process each term
  (add_term(std::forward<Terms>(terms)), ...);

  return Expression(Expression::Operator::custom, std::move(operands));
}

template<
  typename... Terms, 
  typename = std::enable_if_t< !std::disjunction_v< std::is_same< std::decay_t<Terms>, std::vector<Expression> >...> >
>
Expression max(Terms&&... terms) {
  return customOperator("max", std::forward<Terms>(terms)...);
};

inline Expression max(std::vector<Expression> terms) {
  if (terms.empty()) {
    throw std::invalid_argument("CP: max requires at least one element");
  }

  // Construct operands with "max" identifier
  std::vector<Operand> operands;
  operands.reserve(terms.size() + 1);
  operands.push_back(Expression::getCustomIndex("max"));

  // Move terms into operands
  for (auto& term : terms) {
    operands.push_back(std::move(term));
  }

  return Expression(Expression::Operator::custom, std::move(operands));
}


template<
  typename... Terms, 
  typename = std::enable_if_t< !std::disjunction_v< std::is_same< std::decay_t<Terms>, std::vector<Expression> >...> >
>
Expression min(Terms&&... terms) {
  return customOperator("min", std::forward<Terms>(terms)...);
};

inline Expression min(std::vector<Expression> terms) {
  if (terms.empty()) {
    throw std::invalid_argument("CP: min requires at least one element");
  }

  // Construct operands with "min" identifier
  std::vector<Operand> operands;
  operands.reserve(terms.size() + 1);
  operands.push_back(Expression::getCustomIndex("min"));

  // Move terms into operands
  for (auto& term : terms) {
    operands.push_back(std::move(term));
  }

  return Expression(Expression::Operator::custom, std::move(operands));
}

inline Expression if_then_else(Expression condition, Expression ifExpression, Expression elseExpression) {
  std::vector< Operand > operands;

  operands.push_back( Expression::getCustomIndex("if_then_else") );
  operands.push_back(std::move(condition));
  operands.push_back(std::move(ifExpression));
  operands.push_back(std::move(elseExpression));

  return Expression(Expression::Operator::custom,std::move(operands));
};

using Cases = std::vector< std::pair<Expression, Expression> >;
/**
 * @brief Creates an n-ary expression composed of a collection condition-expression pairs representing:
 * ```
 * if condition_1 then 
 *   expression_1
 * else if condition_2 then
 *   expression_2
 * ...
 * else if condition_n then
 *   expression_n
 * end if
 * ```
 */
inline Expression n_ary_if(Cases cases, Expression elseExpression) {
  std::vector< Operand > operands;

  operands.push_back( Expression::getCustomIndex("n_ary_if") );
  for ( auto& [condition,expression] : cases ) {
    operands.push_back(std::move(condition));
    operands.push_back(std::move(expression));
  }
  operands.push_back(std::move(elseExpression));

  return Expression(Expression::Operator::custom,std::move(operands));
};

/*******************************************
 * Model
 ******************************************/

/**
 * @brief Represents a model of a constraint program.
 */
class Model {
public:
  enum class ObjectiveSense { FEASIBLE, MINIMIZE, MAXIMIZE };
  inline Model(ObjectiveSense objectiveSense = ObjectiveSense::FEASIBLE ) : objectiveSense(objectiveSense) {};
  inline ObjectiveSense getObjectiveSense() const { return objectiveSense; };
  inline const Expression& getObjective() const { return objective; };
  inline const std::deque< Variable >& getVariables() const { return variables; };
  inline const std::deque< IndexedVariables >& getIndexedVariables() const { return indexedVariables; };
  inline const std::deque< Expression >& getConstraints() const { return constraints; };
  inline const std::deque< Sequence >& getSequences() const { return sequences; };

  inline const Expression& setObjective(Expression objective) { this->objective = std::move(objective); return this->objective; };

  inline const Variable& addVariable( Variable::Type type, std::string name, double lowerBound, double upperBound ) {
    variables.emplace_back(type, std::move(name), lowerBound, upperBound);
    return variables.back();
  };

  inline IndexedVariables& addIndexedVariables( Variable::Type type, std::string name ) {
    indexedVariables.emplace_back(type, std::move(name));
    return indexedVariables.back();
  };

  inline const Variable& addBinaryVariable(std::string name) {
    variables.emplace_back(Variable::Type::BOOLEAN, std::move(name), 0, 1);
    return variables.back();
  };

  inline const Variable& addIntegerVariable(std::string name) {
    variables.emplace_back(Variable::Type::INTEGER, std::move(name), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
    return variables.back();
  };

  inline const Variable& addRealVariable(std::string name) {
    variables.emplace_back(Variable::Type::REAL, std::move(name), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max());
    return variables.back();
  };

  inline const Sequence& addSequence(std::string name, size_t n) {
    sequences.emplace_back(name,n);
    return sequences.back();
  }

  inline const Variable& addVariable( Variable::Type type, std::string name, Expression expression ) {
    variables.emplace_back(type, std::move(name), std::move(expression));
    return variables.back();
  }

  inline const Expression& addConstraint( Expression constraint) {
    constraints.push_back( std::move(constraint) );
    return constraints.back();
  };

  inline std::string stringify() const {
    std::string result;
    result +=  "Sequences:\n";
    for (const auto& sequence : getSequences()) {
      result += sequence.stringify() + "\n";
    }
    result += "Variables:\n";
    for (const auto& variable : getVariables()) {
      result += variable.stringify() + "\n";
    }
    result += "Indexed variables:\n";
    for (const auto& indexedVariables : getIndexedVariables()) {
      result += indexedVariables.stringify() + "\n";
    }
    result +=  "Constraints:\n";
    for (const auto& constraint : getConstraints()) {
      result += constraint.stringify() + "\n";
    }
    return result;
  }

private:  
  ObjectiveSense objectiveSense;
  Expression objective;
  std::deque< Sequence > sequences;
  std::deque< Variable > variables;
  std::deque< IndexedVariables > indexedVariables;
  std::deque< Expression > constraints;
};

/*******************************************
 * Solution
 ******************************************/
class Solution;

inline double max(const Solution* solution, const std::vector<Operand>& operands);
inline double min(const Solution* solution, const std::vector<Operand>& operands);
inline double n_ary_if(const Solution* solution, const std::vector<Operand>& operands);
inline double sum(const Solution* solution, const std::vector<Operand>& operands);
inline double avg(const Solution* solution, const std::vector<Operand>& operands);

/**
 * @brief Represents a solution of a constraint programming model.
 */
class Solution {
public:
  Solution(const Model& model);
  const Model& model;
  inline void setObjectiveValue(std::optional<double> value) { _objective = value; };
  inline std::optional<double> getObjectiveValue() const { return _objective; };
  inline void setSequenceValues(const Sequence& sequence, std::vector<double> values);

  inline const std::vector<double>& getSequenceValues(const Sequence& sequence) const;
  inline void setVariableValue(const Variable& variable, double value);
  inline double getVariableValue(const Variable& variable) const;

  inline void addEvaluator( const std::string& name, std::function< double(const Solution*, const std::vector<Operand>&) > implementation );

  inline double evaluate(const Operand& term) const;
  inline double evaluate(const Expression& expression) const;

  inline std::string validate() const;
  inline std::string stringify() const;
  inline std::string stringify(const Variable& variable) const;
private:
  std::optional<double> _objective;
  std::unordered_map< const Sequence*, std::vector<double> > _sequenceValues;
  std::unordered_map< const Variable*, double > _variableValues;
  std::vector< std::function< double(const Solution*, const std::vector<Operand>&) > > _customEvaluators;
};

inline Solution::Solution(const Model& model) : model(model) {
  addEvaluator("max", static_cast<double(*)(const Solution*, const std::vector<Operand>&)>(max));
  addEvaluator("min", static_cast<double(*)(const Solution*, const std::vector<Operand>&)>(min));
  addEvaluator("if_then_else", [](const Solution* solution, const std::vector<Operand>& operands) { 
    return solution->evaluate(operands[1]) ? solution->evaluate(operands[2]) : solution->evaluate(operands[3]);
  });
  addEvaluator("n_ary_if", static_cast<double(*)(const Solution*, const std::vector<Operand>&)>(n_ary_if));
  addEvaluator("sum", static_cast<double(*)(const Solution*, const std::vector<Operand>&)>(sum));
  addEvaluator("avg", static_cast<double(*)(const Solution*, const std::vector<Operand>&)>(avg));
  addEvaluator("count", []([[maybe_unused]]const Solution* solution, const std::vector<Operand>& operands) { 
    return operands.size()-1;
  });
  addEvaluator("pow", [](const Solution* solution, const std::vector<Operand>& operands) { 
    return std::pow(solution->evaluate(operands[1]), solution->evaluate(operands[2]));
  });
};

inline void Solution::setSequenceValues(const Sequence& sequence, std::vector<double> values) {
  if ( sequence.variables.size() != values.size() ) {
    throw std::invalid_argument("CP: illegal number of sequence values");
  }
  for (unsigned int i = 0; i < values.size(); i++) {
    _variableValues[&sequence.variables[i]] = values[i];
  }
  _sequenceValues[&sequence] = std::move(values);
};

inline const std::vector<double>& Solution::getSequenceValues(const Sequence& sequence) const {
  return _sequenceValues.at(&sequence);
};

inline void Solution::setVariableValue(const Variable& variable, double value) {
  _variableValues[&variable] = value;
};

inline double Solution::getVariableValue(const Variable& variable) const {
  return _variableValues.at(&variable);
};


inline void Solution::addEvaluator( const std::string& name, std::function< double(const Solution*, const std::vector<Operand>&) > implementation ) {
  auto index = Expression::getCustomIndex(name);
  if ( index >= _customEvaluators.size() ) {
    _customEvaluators.resize(index+1);
  }
  _customEvaluators[index] = std::move(implementation);
}

inline double Solution::evaluate(const Operand& term) const {
  if (std::holds_alternative<double>(term)) {
    return std::get<double>(term);
  }
  else if (std::holds_alternative<std::reference_wrapper<const CP::Variable>>(term)) {
    auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(term).get();
    if ( variable.deducedFrom ) {
      return evaluate(*variable.deducedFrom);
    }
    else if ( variable.lowerBound == variable.upperBound ) {
      return variable.lowerBound;
    }
    return getVariableValue(variable);
  }
  else if ( std::holds_alternative<Expression>(term) ) {
    auto& expression = std::get<Expression>(term);
    return evaluate(expression);
  }
  else {
    throw std::logic_error("CP: unexpected operand");
  }
};

inline double Solution::evaluate(const Expression& expression) const {
  auto& operands = expression.operands;
  using enum Expression::Operator;
  switch (expression._operator) {
    case none:
    {
      return evaluate(operands[0]);
    }
    case negate:
    {
      return -evaluate(operands[0]);
    }
    case logical_not:
    {
      return !evaluate(operands[0]);
    }
    case logical_and:
    {
      return evaluate(operands[0]) && evaluate(operands[1]);
    }
    case logical_or:
    {
      return evaluate(operands[0]) || evaluate(operands[1]);
    }
    case add:
    {
      return evaluate(operands[0]) + evaluate(operands[1]);
    }
    case subtract:
    {
      return evaluate(operands[0]) - evaluate(operands[1]);
    }
    case multiply:
    {
      return evaluate(operands[0]) * evaluate(operands[1]);
    }
    case divide:
    {
      return evaluate(operands[0]) / evaluate(operands[1]);
    }
    case custom:
    {
      auto index = std::get<size_t>(operands.front());
      return _customEvaluators.at(index)(this,operands);
    }
    case less_than:
    {
      return (evaluate(operands[0]) < evaluate(operands[1]));
    }
    case less_or_equal:
    {
      return (evaluate(operands[0]) <= evaluate(operands[1]));
    }
    case greater_than:
    {
      return (evaluate(operands[0]) > evaluate(operands[1]));
    }
    case greater_or_equal:
    {
      return (evaluate(operands[0]) >= evaluate(operands[1]));
    }
    case equal:
    {
      return (evaluate(operands[0]) == evaluate(operands[1]));
    }
    case not_equal:
    {
      return (evaluate(operands[0]) != evaluate(operands[1]));
    }
    default:
    {
      throw std::logic_error("CP: unexpected operator");
    }
  }
};


inline std::string Solution::validate() const {
  std::string result;
  for (const auto& constraint : model.getConstraints()) {
    if ( ! evaluate(constraint) ) {
      result += "infeasible: " + constraint.stringify() + "\n";        
    }
  }
  auto objective = evaluate( model.getObjective() );
  if ( !_objective.has_value() ) {
    result +=  "missing objective, expected: " + std::format("{:.6f}", objective);        
  }
  else if ( 
    _objective.value() < objective - std::numeric_limits<double>::epsilon() ||
    _objective.value() > objective + std::numeric_limits<double>::epsilon()
  )
  {
    result +=  "wrong objective, expected: " + std::format("{:.6f}", objective);        
  }
  else {
    result +=  "objective: " + std::format("{:.6f}", objective);        
  }
  return result;
};

inline std::string Solution::stringify(const Variable& variable) const {
  std::string result = variable.name + " = ";
  try {
    result += std::to_string( evaluate(variable) ) + "\n";
  }
  catch (...) {
    result += "n/a\n";
  }
  return result;
};


inline std::string Solution::stringify() const { 
  std::string result;

  for ( auto& sequence : model.getSequences() ) {
    for ( auto& variable : sequence.variables ) {
      result += stringify(variable);
    }
  }

  for ( auto& variable : model.getVariables()) {
    result += stringify(variable);
  }

  for (const auto& indexedVariables : model.getIndexedVariables()) {
    for ( auto& variable : indexedVariables ) {
      result += stringify(variable);
    }
  }
  result += "objective: " + (_objective.has_value() ? std::to_string(_objective.value()) : "n/a");
  return result;
}

inline double max(const Solution* solution, const std::vector<Operand>& operands) {
  double value = std::numeric_limits<double>::min();
  for (auto& operand : operands | std::views::drop(1)) {
    auto other = solution->evaluate(operand);
    if ( other > value ) {
      value = other;
    }
  }
  return value;
}

inline double min(const Solution* solution, const std::vector<Operand>& operands) {
  double value = std::numeric_limits<double>::max();
  for (auto& operand : operands | std::views::drop(1)) {
    auto other = solution->evaluate(operand);
    if ( other < value ) {
      value = other;
    }
  }
  return value;
}

inline double n_ary_if(const Solution* solution, const std::vector<Operand>& operands) {
  for (size_t index = 1; index < operands.size()-1; index += 2) {
    if ( solution->evaluate(operands[index]) ) {
      return solution->evaluate(operands[index+1]);
    }
  }
  return solution->evaluate(operands.back());
}

inline double sum(const Solution* solution, const std::vector<Operand>& operands) {
  double value = 0.0;
  for (auto& operand : operands | std::views::drop(1)) {
    value += solution->evaluate(operand);
  }
  return value;
}

inline double avg(const Solution* solution, const std::vector<Operand>& operands) {
  double value = 0.0;
  for (auto& operand : operands | std::views::drop(1)) {
    value += solution->evaluate(operand);
  }
  return value / (operands.size()-1);
}

} // end namespace CP
