 /**
 ******************************************************************************
 *
 *  Constraint programming interface
 *
 ******************************************************************************
 */

#pragma once

#include <memory>
#include <list>
#include <vector>
#include <limits>
#include <string>
#include <format>
#include <ranges>
#include <variant>

namespace CP {

struct Expression;

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
  std::list<Variable> _variables;
  reference_vector<Variable> _references;
};

inline std::string IndexedVariable::stringify() const { return container.name + "[" + index.name + "]"; }


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
  inline Expression(Operator _operator, const std::vector< std::variant< size_t, double, std::reference_wrapper<const CP::Variable>, Expression> >& operands) : _operator(_operator), operands(operands) {};
/*
  // Variadic template constructor
  template<typename... Terms>
  inline Expression(double constant, Terms&&... terms) : constant(constant) { (this->terms.push_back(terms), ...); }
*/    
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

  inline std::string stringify() const {
    std::string result;

    auto stringify_unary = [&](const std::string& op) {
      auto& term = operands.front();
      if (std::holds_alternative<double>(term)) {
        auto constant = std::get<double>(term);
        result += op + ( constant < -std::numeric_limits<double>::epsilon() ? std::format("{:.2f}", constant) : std::format("{:.2f}", std::abs(constant)) );
      }
      else if (std::holds_alternative<std::reference_wrapper<const CP::Variable>>(term)) {
        auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(term).get();
        result += op + variable.name;
      }
      else if ( std::holds_alternative<Expression>(term) ) {
        auto& expression = std::get<Expression>(term);
        result += op + "( " + expression.stringify() + " )";
      }
      else {
        throw std::logic_error("CP: unexpected operand for unaray operator " + op);
      }
    };
    auto stringify_binary = [&](const std::string& op) {
      auto& lhs = operands.front();
      auto& rhs = operands.back();
      if (std::holds_alternative<double>(lhs)) {
        auto constant = std::get<double>(lhs);
        result += ( constant < -std::numeric_limits<double>::epsilon() ? std::format("{:.2f}", constant) : std::format("{:.2f}", std::abs(constant)) );
      }
      else if (std::holds_alternative<std::reference_wrapper<const CP::Variable>>(lhs)) {
        auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(lhs).get();
        result += variable.name;
      }
      else if (std::holds_alternative<Expression>(lhs)) {
        auto& expression = std::get<Expression>(lhs);
        if ( (int)_operator < (int)Operator::less_than ) {
          result += "( " + expression.stringify() + " )";
        }
        else {
          result += expression.stringify();
        }
      }
      else {
        throw std::logic_error("CP: unexpected operands for binary operator " + op);
      }
      result += " " + op + " ";
      if (std::holds_alternative<double>(rhs)) {
        auto constant = std::get<double>(rhs);
        result += ( constant < -std::numeric_limits<double>::epsilon() ? std::format("{:.2f}", constant) : std::format("{:.2f}", std::abs(constant)) );
      }
      else if (std::holds_alternative<std::reference_wrapper<const CP::Variable>>(rhs)) {
        auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(rhs).get();
        result += variable.name;
      }
      else if (std::holds_alternative<Expression>(rhs)) {
        auto& expression = std::get<Expression>(rhs);
        if ( (int)_operator < (int)Operator::less_than && expression._operator != Operator::custom ) {
          result += "( " + expression.stringify() + " )";
        }
        else {
          result += expression.stringify();
        }
      }
      else {
        throw std::logic_error("CP: unexpected operands for binary operator " + op);
      }
    };
    switch (_operator) {
      case Operator::none:
      {
        auto& term = operands.front();
        if ( std::holds_alternative<double>(term) ) {
          auto constant = std::get<double>(term);
          result += ( constant < -std::numeric_limits<double>::epsilon() ? std::format("{:.2f}", constant) : std::format("{:.2f}", std::abs(constant)) );
        }
        else if ( std::holds_alternative<std::reference_wrapper<const CP::Variable>>(term) ) {
          auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(term).get();
          result += variable.name;
        }
        else {
          throw std::logic_error("CP: unexpected operand");
        }
        break;
      }
      case Operator::negate:
      {
        stringify_unary("-");
        break;
      }
      case Operator::logical_not:
      {
        stringify_unary("!");
        break;
      }
      case Operator::logical_and:
      {
        stringify_binary("&&");
        break;
      }
      case Operator::logical_or:
      {
        stringify_binary("||");
        break;
      }
      case Operator::add:
      {
        stringify_binary("+");
        break;
      }
      case Operator::subtract:
      {
        stringify_binary("-");
        break;
      }
      case Operator::multiply:
      {
        stringify_binary("*");
        break;
      }
      case Operator::divide:
      {
        stringify_binary("/");
        break;
      }
      case Operator::less_than:
      {
        stringify_binary("<");
        break;
      }
      case Operator::custom:
      {
        auto index = std::get<size_t>(operands.front());
        result += customOperators[index] + "( ";
        for ( size_t i = 1; i < operands.size(); i++) {
          if (std::holds_alternative<double>(operands[i])) {
            auto constant = std::get<double>(operands[i]);
            result += ( constant < -std::numeric_limits<double>::epsilon() ? std::format("{:.2f}", constant) : std::format("{:.2f}", std::abs(constant)) );
          }
          else if (std::holds_alternative<std::reference_wrapper<const CP::Variable>>(operands[i])) {
            auto& variable = std::get<std::reference_wrapper<const CP::Variable>>(operands[i]).get();
            result += variable.name;
          }
          else if (std::holds_alternative<Expression>(operands[i])) {
            auto& expression = std::get<Expression>(operands[i]);
            if ( (int)_operator < (int)Operator::less_than ) {
              result += expression.stringify();
            }
            else {
              result += expression.stringify();
            }
          }
          else {
            throw std::logic_error("CP: unexpected operands for custom operator " + customOperators[index]);
          }
          result += ", ";
        }
        result.pop_back();
        result.back() = ' ';
        result += ")";
        break;
      }
      case Operator::less_or_equal:
      {
        stringify_binary("<=");
        break;
      }
      case Operator::greater_than:
      {
        stringify_binary(">");
        break;
      }
      case Operator::greater_or_equal:
      {
        stringify_binary(">=");
        break;
      }
      case Operator::equal:
      {
        stringify_binary("==");
        break;
      }
      case Operator::not_equal:
      {
        stringify_binary("!=");
        break;
      }
      default:
      {
        throw std::logic_error("CP: unexpected operator");
      }
    }
    return result;
  };
  
  Operator _operator;
  std::vector< std::variant< size_t, double, std::reference_wrapper<const CP::Variable>, Expression> > operands;
  static std::vector<std::string> customOperators;
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

std::vector<std::string> Expression::customOperators = {};
std::optional<std::pair<Expression, Expression>> isImplication( const Expression& expression ) {
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
 * Variable
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


// Left side operators
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



// Yoda comparisons
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
  std::list<Variable> _variables;
};

template<typename... Terms>
Expression customOperator(const std::string& name, Terms&&... terms) {
  // Static assert to ensure all terms are either arithmetic, CP::Variable, or Expression
  static_assert(((
    std::is_arithmetic_v<std::decay_t<Terms>> || 
    std::is_same_v<std::decay_t<Terms>, CP::Variable> ||
    std::is_same_v<std::decay_t<Terms>, Expression>) && ...),
    "CP: All terms must be a number, variable, or expression"
  );

  std::vector< std::variant< size_t, double, std::reference_wrapper<const CP::Variable>, Expression> > operands;

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

template<typename... Terms>
Expression max(Terms&&... terms) {
  return customOperator("max", std::forward<Terms>(terms)...);
};

template<typename... Terms>
Expression min(Terms&&... terms) {
  return customOperator("min", std::forward<Terms>(terms)...);
};


Expression if_then_else(Expression condition, Expression ifExpression, Expression elseExpression) {
  std::vector< std::variant< size_t, double, std::reference_wrapper<const CP::Variable>, Expression> > operands;

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
Expression n_ary_if(Cases cases, Expression elseExpression) {
  std::vector< std::variant< size_t, double, std::reference_wrapper<const CP::Variable>, Expression> > operands;

  operands.push_back( Expression::getCustomIndex("n_ary_if") );
  for ( auto& [condition,expression] : cases ) {
    operands.push_back(std::move(condition));
    operands.push_back(std::move(expression));
  }
  operands.push_back(std::move(elseExpression));

  return Expression(Expression::Operator::custom,std::move(operands));
};

/**
 * @brief Represents a model of a constraint program.
 */
class Model {
public:
  enum class ObjectiveSense { FEASIBLE, MINIMIZE, MAXIMIZE };
  inline Model(ObjectiveSense objectiveSense = ObjectiveSense::FEASIBLE ) : objectiveSense(objectiveSense) {};
  inline ObjectiveSense getObjectiveSense() const { return objectiveSense; };
  inline const Expression& getObjective() const { return objective; };
  inline const std::list< Variable >& getVariables() const { return variables; };
  inline const std::list< IndexedVariables >& getIndexedVariables() const { return indexedVariables; };
  inline const std::list< Expression >& getConstraints() const { return constraints; };
  inline const std::list< Sequence >& getSequences() const { return sequences; };

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

  inline const reference_vector<const Variable> addSequence(std::string name, size_t n) {
    sequences.emplace_back(name,n);
    return sequences.back().variables;
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
  std::list< Sequence > sequences;
  std::list< Variable > variables;
  std::list< IndexedVariables > indexedVariables;
  std::list< Expression > constraints;
};

} // end namespace CP
