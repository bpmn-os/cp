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

struct LinearTerm;
struct BooleanTerm;

struct Expression {
  virtual ~Expression() {};
  virtual std::string stringify() const = 0;
};

struct LinearExpression;
struct BooleanExpression;
struct MaxExpression;
struct MinExpression;

struct Constraint {
  virtual ~Constraint() {};
  virtual std::string stringify() const = 0;
};

struct LinearConstraint;
struct BooleanConstraint;
struct ConditionalConstraint;

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
  Variable(Type type, std::string name ) 
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
  Variable(Type type, std::string name, double lowerBound, double upperBound ) 
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
  template<typename ExpressionType>
  Variable(Type type, std::string name, const ExpressionType& expression ) 
    : type(type)
    , name(std::move(name))
    , lowerBound( type == Type::BOOLEAN ? 0 : std::numeric_limits<double>::lowest() )
    , upperBound( type == Type::BOOLEAN ? 1 : std::numeric_limits<double>::max() )
  {
    if constexpr (std::is_same_v<ExpressionType, LinearConstraint>) {
        deducedFrom = std::make_unique<BooleanExpression>(expression);
    } else {
        deducedFrom = std::make_unique<ExpressionType>(expression);
    }
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
  
  inline LinearTerm operator*(double coefficient) const;
  inline LinearTerm operator/(double coefficient) const;

  inline LinearExpression operator+(double constant) const;
  inline LinearExpression operator-(double constant) const;
  inline LinearExpression operator+(const Variable& variable) const;
  inline LinearExpression operator-(const Variable& variable) const;
  inline LinearExpression operator+(const LinearTerm& term) const;
  inline LinearExpression operator-(const LinearTerm& term) const;
  inline LinearExpression operator+(LinearExpression expression) const;
  inline LinearExpression operator-(LinearExpression expression) const;

  inline BooleanTerm operator!() const;
  
  inline BooleanExpression operator&&(const Variable& variable) const;
  inline BooleanExpression operator&&(const BooleanTerm& term) const;
  inline BooleanExpression operator&&(const LinearConstraint& term) const;
  inline BooleanExpression operator&&(BooleanExpression expression) const;

  inline BooleanExpression operator||(const Variable& variable) const;
  inline BooleanExpression operator||(const BooleanTerm& term) const;
  inline BooleanExpression operator||(const LinearConstraint& term) const;
  inline BooleanExpression operator||(BooleanExpression expression) const;

  inline LinearConstraint operator==(double constant) const;
  inline LinearConstraint operator==(const Variable& variable) const;
  inline LinearConstraint operator==(const LinearTerm& term) const;
  inline LinearConstraint operator==(const LinearExpression& expression) const;

  inline LinearConstraint operator<=(double constant) const;
  inline LinearConstraint operator<=(const Variable& variable) const;
  inline LinearConstraint operator<=(const LinearTerm& term) const;
  inline LinearConstraint operator<=(const LinearExpression& expression) const;

  inline LinearConstraint operator>=(double constant) const;
  inline LinearConstraint operator>=(const Variable& variable) const;
  inline LinearConstraint operator>=(const LinearTerm& term) const;
  inline LinearConstraint operator>=(const LinearExpression& expression) const;

  inline LinearConstraint operator<(double constant) const;
  inline LinearConstraint operator<(const Variable& variable) const;
  inline LinearConstraint operator<(const LinearTerm& term) const;
  inline LinearConstraint operator<(const LinearExpression& expression) const;

  inline LinearConstraint operator>(double constant) const;
  inline LinearConstraint operator>(const Variable& variable) const;
  inline LinearConstraint operator>(const LinearTerm& term) const;
  inline LinearConstraint operator>(const LinearExpression& expression) const;

  inline BooleanConstraint operator==(bool constant) const;
  inline BooleanConstraint operator!=(bool constant) const;

  inline BooleanConstraint operator==(const BooleanTerm& term) const;
  inline BooleanConstraint operator!=(const BooleanTerm& term) const;

  inline BooleanConstraint operator==(const BooleanExpression& expression) const;
  inline BooleanConstraint operator!=(const BooleanExpression& expression) const;

  inline ConditionalConstraint implies(LinearConstraint constraint) const;
  inline ConditionalConstraint implies(BooleanConstraint constraint) const;

  std::string stringify() const {
    if ( deducedFrom ) {
      return name + " := " + deducedFrom->stringify();
    }
    
    if ( lowerBound == upperBound ) {
      return name + " := " + std::format("{:.2f}", lowerBound);
    }
     
    return name + " ∈ [ " + ( lowerBound == std::numeric_limits<double>::lowest() ? "-infinity" : std::format("{:.2f}", lowerBound) ) + ", " + ( upperBound == std::numeric_limits<double>::max() ? "infinity" : std::format("{:.2f}", upperBound) ) + " ]";
  }

};

template<typename T>
class reference_vector : public std::vector<std::reference_wrapper<T>> {
public:
  // Overloading the [] operator to return a const reference to T
  const T& operator[](std::size_t index) const {
    return std::vector<std::reference_wrapper<T>>::at(index).get();
  }

  // Overloading the [] operator to return a reference to T
  T& operator[](std::size_t index) {
    return std::vector<std::reference_wrapper<T>>::at(index).get();
  }
};

struct IndexedVariables;

struct IndexedVariable {
  IndexedVariable(const IndexedVariables& container, const Variable& index) : container(container), index(index) {}
  const IndexedVariables& container;
  const Variable& index;
  std::string stringify() const;
};

struct IndexedVariables {
  Variable::Type type;
  std::string name;
  IndexedVariables(Variable::Type type, std::string name) : type(type), name(std::move(name)) {} 
  IndexedVariables(const IndexedVariables&) = delete; // Disable copy constructor
  IndexedVariables& operator=(const IndexedVariables&) = delete; // Disable copy assignment

  const Variable& operator[](std::size_t index) const {
    return _references.at(index);
  }

  IndexedVariable operator[](const Variable& index) const {
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
    
  std::string stringify() const {
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

std::string IndexedVariable::stringify() const { return container.name + "[" + index.name + "]"; }

/**
 * @brief Represents a term in a linear expression.
 */
struct LinearTerm {
  LinearTerm(double coefficient, const Variable& variable) : coefficient(coefficient), variable(variable) {};
  double coefficient;
  const Variable& variable;
  inline LinearTerm operator*(double multiplier) const {
     auto result = *this;
     result.coefficient *= multiplier;
     return result; 
  };
  inline LinearTerm operator/(double divisor) const {
     auto result = *this;
     result.coefficient /= divisor;
     return result; 
  };

  inline LinearExpression operator+(double constant) const;
  inline LinearExpression operator-(double constant) const;
  inline LinearExpression operator+(const Variable& variable) const;
  inline LinearExpression operator-(const Variable& variable) const;
  inline LinearExpression operator+(const LinearTerm& term) const;
  inline LinearExpression operator-(const LinearTerm& term) const;
  inline LinearExpression operator+(LinearExpression expression) const;
  inline LinearExpression operator-(LinearExpression expression) const;
  
  inline void operator*=(double multiplier) { coefficient *= multiplier; }
  inline void operator/=(double divisor) { coefficient /= divisor; }

  inline LinearConstraint operator==(double constant) const;
  inline LinearConstraint operator==(const Variable& variable) const;
  inline LinearConstraint operator==(const LinearTerm& term) const;
  inline LinearConstraint operator==(const LinearExpression& expression) const;

  inline LinearConstraint operator<=(double constant) const;
  inline LinearConstraint operator<=(const Variable& variable) const;
  inline LinearConstraint operator<=(const LinearTerm& term) const;
  inline LinearConstraint operator<=(const LinearExpression& expression) const;

  inline LinearConstraint operator>=(double constant) const;
  inline LinearConstraint operator>=(const Variable& variable) const;
  inline LinearConstraint operator>=(const LinearTerm& term) const;
  inline LinearConstraint operator>=(const LinearExpression& expression) const;

  inline LinearConstraint operator<(double constant) const;
  inline LinearConstraint operator<(const Variable& variable) const;
  inline LinearConstraint operator<(const LinearTerm& term) const;
  inline LinearConstraint operator<(const LinearExpression& expression) const;

  inline LinearConstraint operator>(double constant) const;
  inline LinearConstraint operator>(const Variable& variable) const;
  inline LinearConstraint operator>(const LinearTerm& term) const;
  inline LinearConstraint operator>(const LinearExpression& expression) const;

  inline ConditionalConstraint implies(LinearConstraint constraint) const;
};

inline LinearTerm Variable::operator*(double multiplier) const { return LinearTerm(multiplier,*this); }
inline LinearTerm Variable::operator/(double divisor) const { return LinearTerm(1.0/divisor,*this); }
// Left multiplication operator
inline LinearTerm operator*(double multiplier, const Variable& variable) {  return variable * multiplier; };
inline LinearTerm operator*(double multiplier, LinearTerm term) {
  term.coefficient *= multiplier;
  return term;
};

/**
 * @brief Represents a linear expression composed of linear terms and a constant.
 */
struct LinearExpression : Expression {
  LinearExpression() : constant(0.0) {};
  LinearExpression(double constant) : constant(constant) {};
  LinearExpression(const Variable& variable) : constant(0.0) {
    terms.push_back( LinearTerm(1.0,variable) );
  };
  LinearExpression(const LinearTerm& term) : constant(0.0) {
    terms.push_back( std::move(term) );
  };
  // Variadic template constructor
  template<typename... Terms>
  LinearExpression(double constant, Terms&&... terms) : constant(constant) { (this->terms.push_back(terms), ...); }
    
  inline LinearExpression operator*(double multiplier) const {
    auto result = *this;
    for ( auto& term : result.terms ) {
      term.coefficient *= multiplier;
    }
    result.constant *= multiplier;
    return result; 
  };
  inline LinearExpression operator/(double divisor) const {
    auto result = *this;
    for ( auto& term : result.terms ) {
      term.coefficient /= divisor;
    }
    result.constant /= divisor;
    return result; 
  };

  inline LinearExpression operator+(double constant) const {
    LinearExpression result = *this;
    result.constant += constant;
    return result;
  }
  inline LinearExpression operator-(double constant) const {
    LinearExpression result = *this;
    result.constant -= constant;
    return result;
  }
  inline LinearExpression operator+(const Variable& variable) const {
    LinearExpression result = *this;
    result.terms.push_back( LinearTerm(1.0,variable) );
    return result;
  }
  inline LinearExpression operator-(const Variable& variable) const {
    LinearExpression result = *this;
    result.terms.push_back( LinearTerm(-1.0,variable) );
    return result;
  }
  inline LinearExpression operator+(const LinearTerm& term) const {
    LinearExpression result = *this;
    result.terms.push_back( term );
    return result;
  }
  inline LinearExpression operator-(const LinearTerm& term) const {
    LinearExpression result = *this;
    result.terms.push_back( LinearTerm(-term.coefficient,term.variable) );
    return result;
  }
  inline LinearExpression operator+(const LinearExpression& expression) const {
    LinearExpression result = *this;
    for ( auto& term : expression.terms ) {
      result.terms.push_back( term );
    }
    result.constant += expression.constant;
    return result;
  }
  inline LinearExpression operator-(const LinearExpression& expression) const {
    LinearExpression result = *this;
    for ( auto& term : expression.terms ) {
    result.terms.push_back( LinearTerm(-term.coefficient,term.variable) );
    }
    result.constant -= expression.constant;
    return result;
  }

  inline void operator+=(double value) { constant += value; }
  inline void operator-=(double value) { constant -= value; }

  inline void operator+=(const Variable& variable) { terms.push_back(LinearTerm(1.0,variable)); }
  inline void operator-=(const Variable& variable) { terms.push_back(LinearTerm(-1.0,variable)); }

  inline void operator+=(const LinearTerm& term) { terms.push_back(term); }
  inline void operator-=(const LinearTerm& term) { terms.push_back(LinearTerm(-term.coefficient,term.variable)); }

  inline void operator+=(const LinearExpression& expression) {
    for ( auto& term : expression.terms ) {
      terms.push_back( term );
    }
    constant += expression.constant;
  }
  inline void operator-=(const LinearExpression& expression) {
    for ( auto& term : expression.terms ) {
      terms.push_back( LinearTerm(-term.coefficient,term.variable) );
    }
    constant -= expression.constant;
  }

  inline void operator*=(double multiplier) {
    for ( auto& term : terms ) {
      term.coefficient *= multiplier;
    }
    constant *= multiplier;
  }
  inline void operator/=(double divisor) {
    for ( auto& term : terms ) {
      term.coefficient /= divisor;
    }
    constant /= divisor;
  }

  inline LinearConstraint operator==(double constant) const;
  inline LinearConstraint operator==(const Variable& variable) const;
  inline LinearConstraint operator==(const LinearTerm& term) const;
  inline LinearConstraint operator==(const LinearExpression& expression) const;

  inline LinearConstraint operator<=(double constant) const;
  inline LinearConstraint operator<=(const Variable& variable) const;
  inline LinearConstraint operator<=(const LinearTerm& term) const;
  inline LinearConstraint operator<=(const LinearExpression& expression) const;

  inline LinearConstraint operator>=(double constant) const;
  inline LinearConstraint operator>=(const Variable& variable) const;
  inline LinearConstraint operator>=(const LinearTerm& term) const;
  inline LinearConstraint operator>=(const LinearExpression& expression) const;

  inline LinearConstraint operator<(double constant) const;
  inline LinearConstraint operator<(const Variable& variable) const;
  inline LinearConstraint operator<(const LinearTerm& term) const;
  inline LinearConstraint operator<(const LinearExpression& expression) const;

  inline LinearConstraint operator>(double constant) const;
  inline LinearConstraint operator>(const Variable& variable) const;
  inline LinearConstraint operator>(const LinearTerm& term) const;
  inline LinearConstraint operator>(const LinearExpression& expression) const;

  std::string stringify() const override {
    std::string result = ( constant < -std::numeric_limits<double>::epsilon() ? std::format("{:.2f}", constant) : std::format("{:.2f}", std::abs(constant)) );
    for (auto& term : terms) {
      if ( term.coefficient < -std::numeric_limits<double>::epsilon() ) {
        result += " - " + std::format("{:.2f}", -term.coefficient);
      }
      else {
        result += " + " + std::format("{:.2f}", std::abs(term.coefficient));
      }
      result += "*" + term.variable.name;
    }
    return result;
  };
  
  std::list< LinearTerm > terms;
  double constant;
};

inline LinearExpression operator-(LinearExpression expression) {
  expression.constant *= -1;
  for ( auto& term : expression.terms ) {
    term.coefficient *= -1;
  }
  return expression;
}

inline LinearExpression operator*(double multiplier, LinearExpression expression) {
  expression.constant *= multiplier;
  for ( auto& term : expression.terms ) {
    term.coefficient *= multiplier;
  }
  return expression;
};

Variable::Variable(Type type, std::string name, const Variable& other ) 
  : type(type)
  , name(std::move(name))
  , lowerBound( type == Type::BOOLEAN ? 0 : std::numeric_limits<double>::lowest() )
  , upperBound( type == Type::BOOLEAN ? 1 : std::numeric_limits<double>::max() )
  , deducedFrom( std::make_unique<LinearExpression>(other) )
{
};

inline LinearExpression Variable::operator+(double constant) const { return LinearExpression( constant, LinearTerm(1.0,*this) );}
inline LinearExpression Variable::operator-(double constant) const { return LinearExpression( -constant, LinearTerm(1.0,*this) );}
inline LinearExpression Variable::operator+(const Variable& variable) const { return LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(1.0,variable) );}
inline LinearExpression Variable::operator-(const Variable& variable) const { return LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-1.0,variable) );}
inline LinearExpression Variable::operator+(const LinearTerm& term) const { return LinearExpression( 0.0, LinearTerm(1.0,*this), term );}
inline LinearExpression Variable::operator-(const LinearTerm& term) const { return LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-term.coefficient,term.variable) );}
inline LinearExpression Variable::operator+(LinearExpression expression) const { expression.terms.push_front( LinearTerm(1.0,*this) ); return expression; }
inline LinearExpression Variable::operator-(LinearExpression expression) const { expression.terms.push_front( LinearTerm(-1.0,*this) ); return -expression; }

inline LinearExpression LinearTerm::operator+(double constant) const { return LinearExpression( constant, *this );}
inline LinearExpression LinearTerm::operator-(double constant) const { return LinearExpression( -constant, *this );}
inline LinearExpression LinearTerm::operator+(const Variable& variable) const { return LinearExpression( 0.0, *this, LinearTerm(1.0,variable) );}
inline LinearExpression LinearTerm::operator-(const Variable& variable) const { return LinearExpression( 0.0, *this, LinearTerm(-1.0,variable) );}
inline LinearExpression LinearTerm::operator+(const LinearTerm& term) const { return LinearExpression( 0.0, *this, term );}
inline LinearExpression LinearTerm::operator-(const LinearTerm& term) const { return LinearExpression( 0.0, *this, LinearTerm(-term.coefficient,term.variable) );}
inline LinearExpression LinearTerm::operator+(LinearExpression expression) const { expression.terms.push_front( *this ); return expression; }
inline LinearExpression LinearTerm::operator-(LinearExpression expression) const { expression.terms.push_front( LinearTerm(-coefficient,variable) ); return -expression; }

// Left addition/subtraction operators
inline LinearExpression operator+(double constant, const Variable& variable) {  return LinearExpression(constant, LinearTerm(1.0,variable) ); };
inline LinearExpression operator-(double constant, const Variable& variable) {  return LinearExpression(constant, LinearTerm(-1.0,variable) ); };
inline LinearExpression operator+(double constant, const LinearTerm& term) {  return LinearExpression(constant, term ); };
inline LinearExpression operator-(double constant, const LinearTerm& term) {  return LinearExpression(constant, LinearTerm(-term.coefficient,term.variable) ); };
inline LinearExpression operator+(double constant, LinearExpression expression) { expression.constant += constant; return expression; };
inline LinearExpression operator-(double constant, LinearExpression expression) { expression.constant -= constant; return -expression; };

/**
 * @brief Represents a constraint comparing a linear expression with zero.
 */
struct LinearConstraint : Constraint {
  enum class Type { EQUAL, LESSOREQUAL, GREATEROREQUAL, LESSTHAN, GREATERTHAN };
  LinearConstraint(Type type, LinearExpression expression) : type(type), expression(expression) {};
  Type type;
  LinearExpression expression;

  std::string stringify() const override {
    std::string result = expression.stringify();
    if ( type == Type::EQUAL ) {
      result += " == 0";
    }
    else if ( type == Type::LESSOREQUAL ) {
      result += " <= 0";
    }
    else if ( type == Type::GREATEROREQUAL ) {
      result += " >= 0";
    }
    else if ( type == Type::LESSTHAN ) {
      result += " < 0";
    }
    else if ( type == Type::GREATERTHAN ) {
      result += " > 0";
    }
    return result;
  };

};

inline LinearConstraint Variable::operator==(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression( -constant, LinearTerm(1.0,*this) ) ); 
};
inline LinearConstraint Variable::operator==(const Variable& variable) const { 
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint Variable::operator==(const LinearTerm& term) const {
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint Variable::operator==(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearTerm(1.0,*this) - expression );
};

inline LinearConstraint Variable::operator<=(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression( -constant, LinearTerm(1.0,*this) ) ); 
};
inline LinearConstraint Variable::operator<=(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint Variable::operator<=(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint Variable::operator<=(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearTerm(1.0,*this) - expression );
};

inline LinearConstraint Variable::operator>=(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression( -constant, LinearTerm(1.0,*this) ) ); 
};
inline LinearConstraint Variable::operator>=(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint Variable::operator>=(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint Variable::operator>=(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearTerm(1.0,*this) - expression );
};

inline LinearConstraint Variable::operator<(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression( -constant, LinearTerm(1.0,*this) ) ); 
};
inline LinearConstraint Variable::operator<(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint Variable::operator<(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint Variable::operator<(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearTerm(1.0,*this) - expression );
};

inline LinearConstraint Variable::operator>(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression( -constant, LinearTerm(1.0,*this) ) ); 
};
inline LinearConstraint Variable::operator>(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint Variable::operator>(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression( 0.0, LinearTerm(1.0,*this), LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint Variable::operator>(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearTerm(1.0,*this) - expression );
};

inline LinearConstraint LinearTerm::operator==(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression( -constant, *this ) ); 
};
inline LinearConstraint LinearTerm::operator==(const Variable& variable) const { 
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression( 0.0, *this, LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint LinearTerm::operator==(const LinearTerm& term) const {
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression( 0.0, *this, LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint LinearTerm::operator==(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::EQUAL, *this - expression );
};

inline LinearConstraint LinearTerm::operator<=(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression( -constant, *this ) ); 
};
inline LinearConstraint LinearTerm::operator<=(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression( 0.0, *this, LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint LinearTerm::operator<=(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression( 0.0, *this, LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint LinearTerm::operator<=(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, *this - expression );
};

inline LinearConstraint LinearTerm::operator>=(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression( -constant, *this ) ); 
};
inline LinearConstraint LinearTerm::operator>=(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression( 0.0, *this, LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint LinearTerm::operator>=(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression( 0.0, *this, LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint LinearTerm::operator>=(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, *this - expression );
};

inline LinearConstraint LinearTerm::operator<(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression( -constant, *this ) ); 
};
inline LinearConstraint LinearTerm::operator<(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression( 0.0, *this, LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint LinearTerm::operator<(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression( 0.0, *this, LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint LinearTerm::operator<(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, *this - expression );
};

inline LinearConstraint LinearTerm::operator>(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression( -constant, *this ) ); 
};
inline LinearConstraint LinearTerm::operator>(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression( 0.0, *this, LinearTerm(-1.0,variable) ) ); 
};
inline LinearConstraint LinearTerm::operator>(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression( 0.0, *this, LinearTerm(-term.coefficient,term.variable) ) ); 
};
inline LinearConstraint LinearTerm::operator>(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, *this - expression );
};

inline LinearConstraint LinearExpression::operator==(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression(*this) - constant ); 
};
inline LinearConstraint LinearExpression::operator==(const Variable& variable) const { 
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression(*this) - LinearTerm(1.0,variable) ); 
};
inline LinearConstraint LinearExpression::operator==(const LinearTerm& term) const {
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression(*this) - term ); 
};
inline LinearConstraint LinearExpression::operator==(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::EQUAL, LinearExpression(*this) - expression );
};

inline LinearConstraint LinearExpression::operator<=(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression(*this) - constant ); 
};
inline LinearConstraint LinearExpression::operator<=(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression(*this) - LinearTerm(1.0,variable) ); 
};
inline LinearConstraint LinearExpression::operator<=(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression(*this) - term ); 
};
inline LinearConstraint LinearExpression::operator<=(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::LESSOREQUAL, LinearExpression(*this) - expression );
};

inline LinearConstraint LinearExpression::operator>=(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression(*this) - constant ); 
};
inline LinearConstraint LinearExpression::operator>=(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression(*this) - LinearTerm(1.0,variable) ); 
};
inline LinearConstraint LinearExpression::operator>=(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression(*this) - term ); 
};
inline LinearConstraint LinearExpression::operator>=(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::GREATEROREQUAL, LinearExpression(*this) - expression );
};

inline LinearConstraint LinearExpression::operator<(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression(*this) - constant ); 
};
inline LinearConstraint LinearExpression::operator<(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression(*this) - LinearTerm(1.0,variable) ); 
};
inline LinearConstraint LinearExpression::operator<(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression(*this) - term ); 
};
inline LinearConstraint LinearExpression::operator<(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::LESSTHAN, LinearExpression(*this) - expression );
};

inline LinearConstraint LinearExpression::operator>(double constant) const { 
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression(*this) - constant ); 
};
inline LinearConstraint LinearExpression::operator>(const Variable& variable) const {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression(*this) - LinearTerm(1.0,variable) ); 
};
inline LinearConstraint LinearExpression::operator>(const LinearTerm& term) const  {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression(*this) - term ); 
};
inline LinearConstraint LinearExpression::operator>(const LinearExpression& expression) const {
  return LinearConstraint( LinearConstraint::Type::GREATERTHAN, LinearExpression(*this) - expression );
};

// Yoda comparisons
inline LinearConstraint operator==(double constant, const Variable& variable) {  return variable == constant; };
inline LinearConstraint operator==(double constant, const LinearTerm& term) {  return term == constant; };
inline LinearConstraint operator==(double constant, const LinearExpression& expression) {  return expression == constant; };

inline LinearConstraint operator<=(double constant, const Variable& variable) {  return variable >= constant; };
inline LinearConstraint operator<=(double constant, const LinearTerm& term) {  return term >= constant; };
inline LinearConstraint operator<=(double constant, const LinearExpression& expression) {  return expression >= constant; };

inline LinearConstraint operator>=(double constant, const Variable& variable) {  return variable <= constant; };
inline LinearConstraint operator>=(double constant, const LinearTerm& term) {  return term <= constant; };
inline LinearConstraint operator>=(double constant, const LinearExpression& expression) {  return expression <= constant; };

inline LinearConstraint operator<(double constant, const Variable& variable) {  return variable > constant; };
inline LinearConstraint operator<(double constant, const LinearTerm& term) {  return term > constant; };
inline LinearConstraint operator<(double constant, const LinearExpression& expression) {  return expression > constant; };

inline LinearConstraint operator>(double constant, const Variable& variable) {  return variable < constant; };
inline LinearConstraint operator>(double constant, const LinearTerm& term) {  return term < constant; };
inline LinearConstraint operator>(double constant, const LinearExpression& expression) {  return expression < constant; };

/**
 * @brief Represents a term in a logical expression.
 */
struct BooleanTerm {
  /**
   * @brief Constructs a boolean term with a variable and optional negation.
   * 
   * @param variable The variable associated with the boolean term.
   * @param negated Whether the boolean term is negated (default is false).
   */
  BooleanTerm(const Variable& variable, bool negated = false) : variable(variable), negated(negated) {}; 
  const Variable& variable;
  bool negated;
  inline BooleanTerm operator!() const { return BooleanTerm(variable,!negated); };
  inline BooleanExpression operator&&(const Variable& variable) const;
  inline BooleanExpression operator&&(const BooleanTerm& term) const;
  inline BooleanExpression operator&&(const LinearConstraint& term) const;
  inline BooleanExpression operator&&(BooleanExpression expression) const;

  inline BooleanExpression operator||(const Variable& variable) const;
  inline BooleanExpression operator||(const BooleanTerm& term) const;
  inline BooleanExpression operator||(const LinearConstraint& term) const;
  inline BooleanExpression operator||(BooleanExpression expression) const;

  inline BooleanConstraint operator==(bool constant) const;
  inline BooleanConstraint operator!=(bool constant) const;

  inline BooleanConstraint operator==(const Variable& variable) const;
  inline BooleanConstraint operator!=(const Variable& variable) const;

  inline BooleanConstraint operator==(const BooleanTerm& term) const;
  inline BooleanConstraint operator!=(const BooleanTerm& term) const;

  inline BooleanConstraint operator==(const BooleanExpression& expression) const;
  inline BooleanConstraint operator!=(const BooleanExpression& expression) const;

  inline ConditionalConstraint implies(LinearConstraint linearConstraint) const;
  
  std::string stringify() const {
    return(negated ? "!" : "") + variable.name;
  };
};

inline BooleanTerm Variable::operator!() const { return BooleanTerm(*this, true); }


/**
 * @brief Represents a boolean expression applying !, &&, || to boolean expressions, boolean terms, and linear constraints.
 */
struct BooleanExpression : Expression {
  enum class Type { NEGATE, AND, OR, BOOLIFY };
  BooleanExpression(bool value) : type( value ? Type::AND : Type::OR) {}
  BooleanExpression(const LinearConstraint& constraint) : type(Type::BOOLIFY){ this->terms.push_back(constraint); }
  // Variadic template constructor
  template<typename... Terms>
  BooleanExpression(Type type, Terms... terms) : type(type) { (this->terms.push_back(terms), ...); }
  Type type;
  std::list< std::variant< BooleanExpression, BooleanTerm, LinearConstraint > > terms;
  
  inline BooleanExpression operator!() const { 
    return BooleanExpression(Type::NEGATE,*this);
  };

  inline BooleanExpression operator&&(const Variable& variable) {
    if ( type == Type::AND ) {
      auto result = *this;
      result.terms.push_back( BooleanTerm(variable) );
      return result;
    }
    return BooleanExpression(Type::AND,*this,BooleanTerm(variable));
  };
  inline BooleanExpression operator&&(const BooleanTerm& term) {
    if ( type == Type::AND ) {
      auto result = *this;
      result.terms.push_back(term);
      return result;
    }
    return BooleanExpression(Type::AND,*this,term);
  };
  inline BooleanExpression operator&&(const LinearConstraint& term) {
    if ( type == Type::AND ) {
      auto result = *this;
      result.terms.push_back(term);
      return result;
    }
    return BooleanExpression(Type::AND,*this,term);
  };
  inline BooleanExpression operator&&(const BooleanExpression& expression) {
    if ( type == Type::AND && expression.type == Type::AND ) {
      auto result = *this;
      for ( auto& term : expression.terms ) {
        result.terms.push_back(term);
      }
      return result;
    }
    else if ( type == Type::AND ) {
      auto result = *this;
      result.terms.push_back(expression);
      return result;
    }
    else if ( expression.type == Type::AND ) {
      auto result = expression;
      result.terms.push_front(*this);
      return result;
    }
    return BooleanExpression(Type::AND,*this,expression);
  };
  
  inline BooleanExpression operator||(const Variable& variable) {
    if ( type == Type::OR ) {
      auto result = *this;
      result.terms.push_back( BooleanTerm(variable) );
      return result;
    }
    return BooleanExpression(Type::OR,*this,BooleanTerm(variable));
  };
  inline BooleanExpression operator||(const BooleanTerm& term) {
    if ( type == Type::OR ) {
      auto result = *this;
      result.terms.push_back(term);
      return result;
    }
    return BooleanExpression(Type::OR,*this,term);
  };
  inline BooleanExpression operator||(const LinearConstraint& term) {
    if ( type == Type::OR ) {
      auto result = *this;
      result.terms.push_back(term);
      return result;
    }
    return BooleanExpression(Type::OR,*this,term);
  };
  inline BooleanExpression operator||(const BooleanExpression& expression) {
    if ( type == Type::OR && expression.type == Type::OR ) {
      auto result = *this;
      for ( auto& term : expression.terms ) {
        result.terms.push_back(term);
      }
      return result;
    }
    else if ( type == Type::OR ) {
      auto result = *this;
      result.terms.push_back(expression);
      return result;
    }
    else if ( expression.type == Type::OR ) {
      auto result = expression;
      result.terms.push_front(*this);
      return result;
    }
    return BooleanExpression(Type::OR,*this,expression);
  };

  std::string stringify() const override {
    if ( terms.empty() ) {
      return "(nil)";
    }
    if ( type == Type::BOOLIFY ) {
      return "(" + std::visit([](const auto& term) { return term.stringify(); }, terms.front()) + ")";
    }
    if ( type == Type::NEGATE ) {
      return "!" + std::visit([](const auto& term) { return term.stringify(); }, terms.front());
    }
    std::string result = "(";
    result += std::visit([](const auto& term) { return term.stringify(); }, terms.front());
    for (auto term : std::ranges::drop_view(terms, 1) ) {
      result += (std::string) (type == Type::AND ? " && " : " || " ) + std::visit([](const auto& term) { return term.stringify(); }, term);
    }
    result += ")";
    return result;
  };
};

inline BooleanExpression Variable::operator&&(const BooleanTerm& term) const {
  return BooleanExpression( BooleanExpression::Type::AND, BooleanTerm(*this), term );
}
inline BooleanExpression Variable::operator&&(const LinearConstraint& term) const {
  return BooleanExpression( BooleanExpression::Type::AND, BooleanTerm(*this), term );
}

inline BooleanExpression Variable::operator&&(BooleanExpression expression) const {
  if ( expression.type == BooleanExpression::Type::AND ) {
    expression.terms.push_front( BooleanTerm(*this) );
    return expression;
  }
  return BooleanExpression( BooleanExpression::Type::AND, BooleanTerm(*this), expression );
}

inline BooleanExpression BooleanTerm::operator&&(const Variable& variable) const {
  return BooleanExpression( BooleanExpression::Type::AND, *this, BooleanTerm(variable) );
}

inline BooleanExpression BooleanTerm::operator&&(const LinearConstraint& term) const {
  return BooleanExpression( BooleanExpression::Type::AND, *this, term );
}

inline BooleanExpression BooleanTerm::operator&&(BooleanExpression expression) const {
  if ( expression.type == BooleanExpression::Type::AND ) {
    expression.terms.push_front( *this );
    return expression;
  }
  return BooleanExpression( BooleanExpression::Type::AND, *this, expression );
}


inline BooleanExpression Variable::operator||(const BooleanTerm& term) const {
  return BooleanExpression( BooleanExpression::Type::OR, BooleanTerm(*this), term );
}

inline BooleanExpression Variable::operator||(const LinearConstraint& term) const {
  return BooleanExpression( BooleanExpression::Type::OR, BooleanTerm(*this), term );
}

inline BooleanExpression Variable::operator||(BooleanExpression expression) const {
  if ( expression.type == BooleanExpression::Type::OR ) {
    expression.terms.push_front( BooleanTerm(*this) );
    return expression;
  }
  return BooleanExpression( BooleanExpression::Type::OR, BooleanTerm(*this), expression );
}

inline BooleanExpression BooleanTerm::operator||(const Variable& variable) const {
  return BooleanExpression( BooleanExpression::Type::OR, *this, BooleanTerm(variable) );
}

inline BooleanExpression BooleanTerm::operator||(const LinearConstraint& term) const {
  return BooleanExpression( BooleanExpression::Type::OR, *this, term );
}

inline BooleanExpression BooleanTerm::operator||(BooleanExpression expression) const {
  if ( expression.type == BooleanExpression::Type::OR ) {
    expression.terms.push_front( *this );
    return expression;
  }
  return BooleanExpression( BooleanExpression::Type::OR, *this, expression );
}

struct BooleanConstraint : Constraint {
  enum class Type { EQUAL, NOTEQUAL };
  BooleanConstraint(Type type, std::variant<bool,BooleanTerm,BooleanExpression> lhs, std::variant<bool,BooleanTerm,BooleanExpression> rhs) : type(type), lhs(lhs), rhs(rhs) {};
  Type type;
  std::variant<bool,BooleanTerm,BooleanExpression> lhs;
  std::variant<bool,BooleanTerm,BooleanExpression> rhs;
  std::string stringify() const override {
    std::string result;
    if ( std::holds_alternative<BooleanTerm>(lhs) ) {
      result += std::get<BooleanTerm>(lhs).stringify();
    }
    else if ( std::holds_alternative<BooleanExpression>(lhs) ) {
      result += std::get<BooleanExpression>(lhs).stringify();
    }
    else if ( std::holds_alternative<BooleanExpression>(lhs) ) {
      result += std::get<BooleanExpression>(lhs).stringify();
    }
    else {
      result += ( std::holds_alternative<bool>(lhs) ? "true" : "false" );
    }
    result += (type == Type::EQUAL ? " == " : " != ");
    if ( std::holds_alternative<BooleanTerm>(rhs) ) {
      result += std::get<BooleanTerm>(rhs).stringify();
    }
    else if ( std::holds_alternative<BooleanExpression>(rhs) ) {
      result += std::get<BooleanExpression>(rhs).stringify();
    }
    else if ( std::holds_alternative<BooleanExpression>(rhs) ) {
      result += std::get<BooleanExpression>(rhs).stringify();
    }
    else {
      result += ( std::holds_alternative<bool>(rhs) ? "true" : "false" );
    }
    return result;
  };
};

/**
 * @brief Represents a conditional constraint comparing a linear expression with zero if the respective condition holds.
 */
struct ConditionalConstraint : Constraint {
  ConditionalConstraint(BooleanTerm condition, std::variant<LinearConstraint,BooleanConstraint> constraint) : condition(condition), constraint(constraint) {};
  BooleanTerm condition;
  std::variant<LinearConstraint,BooleanConstraint> constraint;
  std::string stringify() const override {
    std::string result = (std::string)"if " + (condition.negated ? "!" : "") + condition.variable.name + " then ";
    if ( std::holds_alternative<LinearConstraint>(constraint) ) {
      result += std::get<LinearConstraint>(constraint).stringify();
    }
    else if ( std::holds_alternative<BooleanConstraint>(constraint) ) {
      result += std::get<BooleanConstraint>(constraint).stringify();
    }
    return result;
  };

};

inline ConditionalConstraint Variable::implies(LinearConstraint linearConstraint) const { return ConditionalConstraint(BooleanTerm(*this),std::move(linearConstraint)); };
inline ConditionalConstraint BooleanTerm::implies(LinearConstraint linearConstraint) const { return ConditionalConstraint(*this,std::move(linearConstraint)); };

/**
 * @brief Represents a collection of integer variables with the property that the variable values are a permutation of {1, ..., n}.
 */
struct Sequence {
  Sequence(std::string name, size_t n) {
    for ( size_t i = 0; i < n; i++ ) {
      _variables.emplace_back(Variable::Type::INTEGER, name + '[' + std::to_string(i) + ']', 1, n );
      variables.push_back( _variables.back() );
    }
  };
  Sequence(const Sequence&) = delete; // Disable copy constructor
  Sequence& operator=(const Sequence&) = delete; // Disable copy assignment
  reference_vector<const Variable> variables;

  std::string stringify() const {
    std::string result = "(";
    for ( const Variable& variable : variables ) {
      result += " " + variable.name + ",";
    }
    result.back() = ' ';
    result += ") is permutation of { 1, ..., " + std::to_string(variables.size()) + " }";
    return result;
  };
private:
  std::list<Variable> _variables;
};



template<typename T>
concept LinearExpressions = std::is_convertible_v<T, LinearExpression>;

/**
 * @brief Represents an expression providing the maximum of a collection of terms and a lower bound value.
 */
 struct MaxExpression : Expression {
  // Variadic template constructor
  template<LinearExpressions... Expressions>
  MaxExpression(Expressions&&... expressions) { (this->expressions.push_back(std::forward<Expressions>(expressions)), ...); }
  MaxExpression(std::list<LinearExpression> expressions) :  expressions(std::move(expressions)) {};

  std::list< LinearExpression > expressions;
  void emplace_back(LinearExpression expression) { expressions.emplace_back(std::move(expression)); };
  std::string stringify() const override {
    std::string result = "max{ " + expressions.front().stringify();
    for (auto expression : std::ranges::drop_view(expressions, 1) ) {
      result += ", " + expression.stringify();
    }
    result += " }";
    return result;
  };
};

template<LinearExpressions... Expressions>
MaxExpression max(Expressions&&... expressions) { return MaxExpression(expressions...); }

template<LinearExpressions... Expressions>
MaxExpression max(CP::MaxExpression maxExpression, Expressions&&... expressions) {
    (maxExpression.expressions.emplace_back(std::forward<Expressions>(expressions)), ...);
    return maxExpression;
}

template<LinearExpressions... Expressions>
MaxExpression max(Expressions&&... expressions, CP::MaxExpression maxExpression) {
    (maxExpression.expressions.emplace_front(std::forward<Expressions>(expressions)), ...);
    return maxExpression;
}


/**
 * @brief Represents an expression providing the minimum of a collection of terms and an upper bound value.
 */
struct MinExpression : Expression {
  // Variadic template constructor
  template<LinearExpressions... Expressions>
  MinExpression(Expressions&&... expressions) { (this->expressions.push_back(std::forward<Expressions>(expressions)), ...); }
  MinExpression(std::list<LinearExpression> expressions) :  expressions(std::move(expressions)) {};

  std::list< LinearExpression > expressions;

  std::string stringify() const override {
    std::string result = "min{ " + expressions.front().stringify();
    for (auto expression : std::ranges::drop_view(expressions, 1) ) {
      result += ", " + expression.stringify();
    }
    result += " }";
    return result;
  };
};

template<LinearExpressions... Expressions>
MinExpression min(Expressions&&... expressions) { return MinExpression(expressions...); }

template<LinearExpressions... Expressions>
MinExpression min(CP::MinExpression minExpression, Expressions&&... expressions) {
    (minExpression.expressions.emplace_back(std::forward<Expressions>(expressions)), ...);
    return minExpression;
}

template<LinearExpressions... Expressions>
MinExpression min(Expressions&&... expressions, CP::MinExpression minExpression) {
    (minExpression.expressions.emplace_front(std::forward<Expressions>(expressions)), ...);
    return minExpression;
}

using Case = std::pair<BooleanTerm,LinearExpression>;

using Cases = std::list< Case >;

/**
 * @brief Represents an n-ary expression composed of a collection condition-expression pairs representing:
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
struct NAryExpression : Expression {
  NAryExpression(BooleanTerm term, LinearExpression ifExpression, LinearExpression elseExpression) : cases({{std::move(term),std::move(ifExpression)}}), elseExpression(std::move(elseExpression)) {};
/*
// Variadic template constructor  
  template<typename... Pairs>
  NAryExpression(LinearExpression elseExpression, Pairs... pairs) : elseExpression(std::move(elseExpression)) {
    static_assert((std::is_same_v<Pairs, std::pair<BooleanTerm, LinearExpression>> && ...), "CP: All arguments must be std::pair<BooleanTerm, LinearExpression>");
    operands = {std::move(pairs)...};
  }
*/
  NAryExpression(Cases cases, LinearExpression elseExpression) : cases(std::move(cases)), elseExpression(std::move(elseExpression)) {};

  Cases cases;
  LinearExpression elseExpression;

  std::string stringify() const override {
    std::string result = (std::string)"if " + (cases.front().first.negated ? "!" : "") + cases.front().first.variable.name + " then " + cases.front().second.stringify();
    for (auto case_ : std::ranges::drop_view(cases, 1) ) {
      result +=  (std::string)" else if " + (case_.first.negated ? "!" : "") + case_.first.variable.name + " then " + case_.second.stringify();
    }
    result += (std::string)" else " + elseExpression.stringify();
    return result;
};

};

/**
 * @brief Constructs an n-ary expression representing an if-then-else condition.
 * 
 * @param term The boolean term of the if condition.
 * @param ifExpression The linear expression if the boolean term is true.
 * @param elseExpression The linear expression if the boolean term is false.
 * @return An NAryExpression representing the if-then-else condition.
 */
inline NAryExpression if_then_else(BooleanTerm term, LinearExpression ifExpression, LinearExpression elseExpression) { return NAryExpression(std::move(term), std::move(ifExpression), std::move(elseExpression)); }
inline NAryExpression n_ary_if(Cases cases, LinearExpression elseExpression) { return NAryExpression(std::move(cases),std::move(elseExpression)); }

/**
 * @brief Represents a model of a constraint program.
 */
class Model {
public:
  enum class ObjectiveSense { FEASIBLE, MINIMIZE, MAXIMIZE };
  Model(ObjectiveSense objectiveSense = ObjectiveSense::FEASIBLE )
   : objectiveSense(objectiveSense)
  {
  };
  inline ObjectiveSense getObjectiveSense() const { return objectiveSense; };
  inline const LinearExpression& getObjective() const { return objective; };
  inline const std::list< Variable >& getVariables() const { return variables; };
  inline const std::list< IndexedVariables >& getIndexedVariables() const { return indexedVariables; };
  inline const std::list< std::variant<LinearConstraint, BooleanConstraint, ConditionalConstraint> >& getConstraints() const { return constraints; };
  inline const std::list< Sequence >& getSequences() const { return sequences; };

  inline const Expression& setObjective(LinearExpression objective) { this->objective = std::move(objective); return this->objective; };

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

  inline const Variable& addVariable( Variable::Type type, std::string name, const Variable& variable ) {
    variables.emplace_back(type, std::move(name), LinearExpression(variable));
    return variables.back();
  }
  
  inline const Variable& addVariable( Variable::Type type, std::string name, const LinearTerm& term ) {
    variables.emplace_back(type, std::move(name), LinearExpression(term));
    return variables.back();
  }

  inline const reference_vector<const Variable> addSequence(std::string name, size_t n) {
    sequences.emplace_back(name,n);
    return sequences.back().variables;
  }

  template<typename ExpressionType>
  inline const Variable& addVariable( Variable::Type type, std::string name, ExpressionType expression ) {
    variables.emplace_back(type, std::move(name), std::move(expression));
    return variables.back();
  }

  inline const LinearConstraint& addConstraint( LinearConstraint constraint) {
    constraints.push_back( std::move(constraint) );
    return std::get<LinearConstraint>(constraints.back());
  };

  inline const BooleanConstraint& addConstraint( BooleanConstraint constraint) {
    constraints.push_back( std::move(constraint) );
    return std::get<BooleanConstraint>(constraints.back());
  };

  inline const ConditionalConstraint& addConstraint( ConditionalConstraint constraint) {
    constraints.push_back( std::move(constraint) );
    return std::get<ConditionalConstraint>(constraints.back());
  };

  std::string stringify() const {
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
        std::visit([&result](const auto& c) { result += c.stringify() + "\n"; }, constraint);
    }
    return result;
  }

private:  
  ObjectiveSense objectiveSense;
  LinearExpression objective;
  std::list< Sequence > sequences;
  std::list< Variable > variables;
  std::list< IndexedVariables > indexedVariables;
  std::list< std::variant<LinearConstraint, BooleanConstraint, ConditionalConstraint> > constraints;
};

} // end namespace CP
