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
#include <limits>
#include <string>
#include <format>
#include <ranges>

namespace CP {

struct LinearTerm;
struct BooleanTerm;


struct Expression {
  virtual ~Expression() = 0;
  virtual std::string stringify() const = 0;
};
Expression::~Expression() {};

struct LinearExpression;
struct AndExpression;
struct OrExpression;
struct MaxExpression;
struct MinExpression;

struct LinearConstraint;
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
   * @brief Constructs a variable with given type and bounds.
   * 
   * @param type The type of the variable (BOOLEAN, INTEGER, REAL).
   * @param lowerBound The lower bound of the variable.
   * @param upperBound The upper bound of the variable.
   */
  Variable(Type type, double lowerBound, double upperBound, std::string name ) 
    : type(type)
    , lowerBound(lowerBound)
    , upperBound(upperBound)
    , name(std::move(name))
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
    , lowerBound( type == Type::BOOLEAN ? 0 : std::numeric_limits<double>::lowest() )
    , upperBound( type == Type::BOOLEAN ? 1 : std::numeric_limits<double>::max() )
    , name(std::move(name))
    , deducedFrom( std::make_unique<ExpressionType>(expression) )
  {
  }

  Variable(const Variable&) = delete; // Disable copy constructor
  Variable& operator=(const Variable&) = delete; // Disable copy assignment
    
  Type type;
  double lowerBound;
  double upperBound;
  std::string name;
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
  
  inline AndExpression operator&&(const Variable& variable) const;
  inline AndExpression operator&&(const BooleanTerm& term) const;
  inline AndExpression operator&&(AndExpression expression) const;

  inline OrExpression operator||(const Variable& variable) const;
  inline OrExpression operator||(const BooleanTerm& term) const;
  inline OrExpression operator||(OrExpression expression) const;

  inline ConditionalConstraint implies(LinearConstraint linearConstraint) const;

  std::string stringify() const {
    if ( deducedFrom ) {
      return name + " = " + deducedFrom->stringify();
    }
    return name + " ∈ [ " + ( lowerBound == std::numeric_limits<double>::lowest() ? "-infinity" : std::format("{:.2f}", lowerBound) ) + ", " + ( upperBound == std::numeric_limits<double>::max() ? "infinity" : std::format("{:.2f}", upperBound) ) + " ]";
  }

};

/**
 * @brief Represents a term in a linear expression.
 */
struct LinearTerm {
  LinearTerm(double coefficient, const Variable& variable) : coefficient(coefficient), variable(variable) {};
  double coefficient;
  const Variable& variable;
  inline LinearTerm operator*(double multiplier) {
     auto result = *this;
     result.coefficient *= multiplier;
     return result; 
  };
  inline LinearTerm operator/(double divisor) {
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
};

//inline LinearTerm operator-(const LinearTerm& term) { return LinearTerm(-term.coefficient,term.variable); }

inline LinearTerm Variable::operator*(double coefficient) const { return LinearTerm(coefficient,*this); }
inline LinearTerm Variable::operator/(double coefficient) const { return LinearTerm(1.0/coefficient,*this); }
// Left multiplication operator
inline LinearTerm operator*(double coefficient, const Variable& variable) {  return variable * coefficient; };
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
    
  inline LinearExpression operator*(double multiplier) {
    auto result = *this;
    for ( auto& term : result.terms ) {
      term.coefficient *= multiplier;
    }
    result.constant *= multiplier;
    return result; 
  };
  inline LinearExpression operator/(double divisor) {
    auto result = *this;
    for ( auto& term : result.terms ) {
      term.coefficient /= divisor;
    }
    result.constant /= divisor;
    return result; 
  };

  inline LinearExpression operator+(double constant) {
    LinearExpression result = *this;
    result.constant += constant;
    return result;
  }
  inline LinearExpression operator-(double constant) {
    LinearExpression result = *this;
    result.constant -= constant;
    return result;
  }
  inline LinearExpression operator+(const Variable& variable) {
    LinearExpression result = *this;
    result.terms.push_back( LinearTerm(1.0,variable) );
    return result;
  }
  inline LinearExpression operator-(const Variable& variable) {
    LinearExpression result = *this;
    result.terms.push_back( LinearTerm(-1.0,variable) );
    return result;
  }
  inline LinearExpression operator+(const LinearTerm& term) {
    LinearExpression result = *this;
    result.terms.push_back( term );
    return result;
  }
  inline LinearExpression operator-(const LinearTerm& term) {
    LinearExpression result = *this;
    result.terms.push_back( LinearTerm(-term.coefficient,term.variable) );
    return result;
  }
  inline LinearExpression operator+(const LinearExpression& expression) {
    LinearExpression result = *this;
    for ( auto& term : expression.terms ) {
      result.terms.push_back( term );
    }
    result.constant += expression.constant;
    return result;
  }
  inline LinearExpression operator-(const LinearExpression& expression) {
    LinearExpression result = *this;
    for ( auto& term : expression.terms ) {
    result.terms.push_back( LinearTerm(-term.coefficient,term.variable) );
    }
    result.constant -= expression.constant;
    return result;
  }

  std::string stringify() const override {
    std::string result = std::format("{:.2f}", constant);
    for (auto& term : terms) {
      if ( term.coefficient < 0 ) {
        result += " - " + std::format("{:.2f}", -term.coefficient);
      }
      else {
        result += " + " + std::format("{:.2f}", term.coefficient);
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
inline LinearExpression LinearTerm::operator-(LinearExpression expression) const { expression.terms.push_front( LinearTerm(-1.0,variable) ); return -expression; }

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
struct LinearConstraint {
  enum class Type { EQUAL, LESSOREQUAL, GREATEOREQUAL };
  Type type;
  LinearExpression expression;
};

struct ConditionalConstraint;

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
  inline AndExpression operator&&(const Variable& variable) const;
  inline AndExpression operator&&(const BooleanTerm& term) const;
  inline AndExpression operator&&(AndExpression expression) const;

  inline OrExpression operator||(const Variable& variable) const;
  inline OrExpression operator||(const BooleanTerm& term) const;
  inline OrExpression operator||(OrExpression expression) const;

  inline ConditionalConstraint implies(LinearConstraint linearConstraint) const;
};

inline BooleanTerm Variable::operator!() const { return BooleanTerm(*this, true); }

/**
 * @brief Represents a conditional constraint comparing a linear expression with zero if the respective condition holds.
 */
struct ConditionalConstraint {
  BooleanTerm condition;
  LinearConstraint linearConstraint;
};

inline ConditionalConstraint Variable::implies(LinearConstraint linearConstraint) const { return ConditionalConstraint(BooleanTerm(*this),std::move(linearConstraint)); };
inline ConditionalConstraint BooleanTerm::implies(LinearConstraint linearConstraint) const { return ConditionalConstraint(*this,std::move(linearConstraint)); };

/**
 * @brief Represents an logical AND expression.
 */
 struct AndExpression : Expression {
  // Variadic template constructor
  template<typename... Terms>
  AndExpression(Terms... terms) { (this->terms.push_back(terms), ...); }
  std::list< BooleanTerm > terms;
  
  inline AndExpression operator&&(const Variable& variable) {
    auto result = *this;
    result.terms.push_back( BooleanTerm(variable) );
    return result;
  };
  inline AndExpression operator&&(const BooleanTerm& term) {
    auto result = *this;
    result.terms.push_back(term);
    return result;
  };
  inline AndExpression operator&&(const AndExpression& expression) {
    auto result = *this;
    for ( auto& term : expression.terms ) {
      result.terms.push_back(term);
    }
    return result;
  };
  
  std::string stringify() const override {
    std::string result = (terms.front().negated ? "!" : "") + terms.front().variable.name;
    for (auto term : std::ranges::drop_view(terms, 1) ) {
      result += (std::string)" && " + (term.negated ? "!" : "") + term.variable.name;
    }
    return result;
  };

};

inline AndExpression Variable::operator&&(const BooleanTerm& term) const {
  return AndExpression( BooleanTerm(*this), term );
}

inline AndExpression Variable::operator&&(AndExpression expression) const {
  expression.terms.push_front( BooleanTerm(*this) );
  return expression;
}

inline AndExpression BooleanTerm::operator&&(const Variable& variable) const {
  return AndExpression(*this,BooleanTerm(variable));
}

inline AndExpression BooleanTerm::operator&&(AndExpression expression) const {
  expression.terms.push_front( *this );
  return expression;
}

/**
 * @brief Represents an logical OR expression.
 */
struct OrExpression : Expression {
  // Variadic template constructor
  template<typename... Terms>
  OrExpression(Terms... terms) { (this->terms.push_back(terms), ...); }
  std::list< BooleanTerm > terms;
  
  inline OrExpression operator||(const Variable& variable) {
    auto result = *this;
    result.terms.push_back( BooleanTerm(variable) );
    return result;
  };
  inline OrExpression operator||(const BooleanTerm& term) {
    auto result = *this;
    result.terms.push_back(term);
    return result;
  };
  inline OrExpression operator||(const OrExpression& expression) {
    auto result = *this;
    for ( auto& term : expression.terms ) {
      result.terms.push_back(term);
    }
    return result;
  };

  std::string stringify() const override {
    std::string result = (terms.front().negated ? "!" : "") + terms.front().variable.name;
    for (auto term : std::ranges::drop_view(terms, 1) ) {
      result += (std::string)" || " + (term.negated ? "!" : "") + term.variable.name;
    }
    return result;
  };

};

inline OrExpression Variable::operator||(const BooleanTerm& term) const {
  return OrExpression( BooleanTerm(*this), term );
}

inline OrExpression Variable::operator||(OrExpression expression) const {
  expression.terms.push_front( BooleanTerm(*this) );
  return expression;
}

inline OrExpression BooleanTerm::operator||(const Variable& variable) const {
  return OrExpression(*this,BooleanTerm(variable));
}

inline OrExpression BooleanTerm::operator||(OrExpression expression) const {
  expression.terms.push_front( *this );
  return expression;
}


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
  NAryExpression(BooleanTerm term, LinearExpression ifExpression, LinearExpression elseExpression) : operands({{std::move(term),std::move(ifExpression)}}), elseExpression(std::move(elseExpression)) {};
/*
// Variadic template constructor  
  template<typename... Pairs>
  NAryExpression(LinearExpression elseExpression, Pairs... pairs) : elseExpression(std::move(elseExpression)) {
    static_assert((std::is_same_v<Pairs, std::pair<BooleanTerm, LinearExpression>> && ...), "CP: All arguments must be std::pair<BooleanTerm, LinearExpression>");
    operands = {std::move(pairs)...};
  }
*/
  NAryExpression(std::list< std::pair<BooleanTerm,LinearExpression> > cases, LinearExpression elseExpression) : operands(std::move(cases)), elseExpression(std::move(elseExpression)) {};

  std::list< std::pair<BooleanTerm,LinearExpression> > operands;
  LinearExpression elseExpression;

  std::string stringify() const override {
    std::string result = (std::string)"if " + (operands.front().first.negated ? "!" : "") + operands.front().first.variable.name + " then " + operands.front().second.stringify();
    for (auto operand : std::ranges::drop_view(operands, 1) ) {
      result +=  (std::string)" else if " + (operand.first.negated ? "!" : "") + operand.first.variable.name + " then " + operand.second.stringify();
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
NAryExpression if_then_else(BooleanTerm term, LinearExpression ifExpression, LinearExpression elseExpression) { return NAryExpression(std::move(term), std::move(ifExpression), std::move(elseExpression)); }
NAryExpression n_ary_if(std::list< std::pair<BooleanTerm,LinearExpression> > cases, LinearExpression elseExpression) { return NAryExpression(std::move(cases),std::move(elseExpression)); }

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
  inline const std::list< LinearConstraint >& getLinearConstraints() const { return linearConstraints; };
  inline const std::list< ConditionalConstraint >& getConditionalConstraints() const { return conditionalConstraints; };

  inline const Expression& setObjective(LinearExpression objective) { this->objective = std::move(objective); return this->objective; };

  inline const Variable& addVariable( Variable::Type type, std::string name, double lowerBound, double upperBound ) {
    variables.emplace_back(type, lowerBound, upperBound, std::move(name));
    return variables.back();
  };

  inline const Variable& addBinaryVariable(std::string name) {
    variables.emplace_back( Variable::Type::BOOLEAN, 0, 1, std::move(name) );
    return variables.back();
  };

  inline const Variable& addIntegerVariable(std::string name) {
    variables.emplace_back( Variable::Type::INTEGER, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(), std::move(name) );
    return variables.back();
  };

  inline const Variable& addRealVariable(std::string name) {
    variables.emplace_back( Variable::Type::REAL, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(), std::move(name) );
    return variables.back();
  };

  template<typename ExpressionType>
  inline const Variable& addVariable( Variable::Type type, std::string name, ExpressionType expression ) {
    variables.emplace_back( type,std::move(name),std::move(expression) );
    return variables.back();
  }

  inline const LinearConstraint& addLinearConstraint( LinearConstraint linearConstraint) {
    linearConstraints.push_back( std::move(linearConstraint) );
    return linearConstraints.back();
  };

  inline const ConditionalConstraint& addConditionalConstraint( ConditionalConstraint conditionalConstraint) {
    conditionalConstraints.push_back( std::move(conditionalConstraint) );
    return conditionalConstraints.back();
  };

private:  
  ObjectiveSense objectiveSense;
  LinearExpression objective;
  std::list< Variable > variables;
  std::list< LinearConstraint > linearConstraints;
  std::list< ConditionalConstraint > conditionalConstraints;
  
};

} // end namespace CP