# Constraint Programming Interface (CP)

This header-only library defines a C++ interface for modeling constraint programming problems. It provides constructs for variables, expressions, sequences, constraints, and solutions, enabling the construction and manipulation of constraint models in an intuitive way.

## Features

- Boolean, integer, and real Variables
- Indexed variables for array-like structures
- Permutation sequences
- Expressions combining variables and constants using arithmetic, logical, and relational operators
- Declarative constraint and objective specification
- Expression evaluation with custom evaluators
- Human-readable stringification of models and solutions

## Components

### Model

The `Model` class holds all variables and constraints as well as the (optional) objective.

```cpp
CP::Model model;
```

### Variables

Unconstrained variables can be added to a model using dedicated methods.

```cpp
inline const Variable& addRealVariable(std::string name);
inline const Variable& addIntegerVariable(std::string name);
inline const Variable& addBinaryVariable(std::string name);
```

Examples:
```cpp
auto& x = model.addRealVariable("x");     // x ∈ [ -infinity, infinity ]
auto& z = model.addIntegerVariable("z");  // z ∈ { -infinity, ..., infinity }
auto& y = model.addBinaryVariable("y");   // y ∈ { false, true }
```

Variables can be added to a model by specifying the type (`CP::Variable::Type::REAL`,`CP::Variable::Type::INTEGER`,`CP::Variable::Type::BOOLEAN`) and lower and upper bounds.

```cpp
inline const Variable& addVariable( Variable::Type type, std::string name, double lowerBound, double upperBound );
```

A model may contain dedicated variables which can deduced from expressions. For such variables, the type and expression must be specified.
```cpp
inline const Variable& addVariable( Variable::Type type, std::string name, Expression expression );
```

### Indexed variables

The struct `IndexedVariables` is used to represent a collection of variables that can be accessed via indices. Indices can be literals, variables, or expressions.
An indexed variable can be added to a model by specifying the type of the contained variables. Before an indexed variable can be used in expressions, the contained variables must be added. This can be done be either providing lower and upper bound or an expression.

```cpp
auto& a = model.addIndexedVariables(CP::Variable::Type::INTEGER, "a");
a.emplace_back( 0, 5 );      // a[0] ∈ { 0, ..., 5 }
a.emplace_back( x + 4 );     // a[1] := x + 4.00 ( x must have been added to the model before )
a.emplace_back( a[1] + 5 );  // a[2] := a[1] + 5.00 ( only variables with lower index must be used )
```

### Sequences

The struct `Sequence` represents a permutation of variables with a given size.

```cpp
auto& s = model.addSequence("s", 3 ); // ( s[0], s[1], s[2] ) is permutation of { 1, ..., 3 }
```


### Expressions

Expressions can be intuitively written using the variable references previously added to a model.

- Arithmetic expressions, e.g. `x + 3 * y?
- Logical expression, e.g. `x && ( y || !z )`
- Comparisons, e.g. `x < y`
- Implications, e.g. `x.implies(y < 5)`
- Min/max expressions, e.g `CP::min( 0, x, 3 * z )`, `CP::max( 0.0, x, 3 * z )`
- If-then-else expressions, e.g. `CP::if_then_else( y, x, 3 * z )`, `CP::n_ary_if( {{y, x}, {!y, 5}}, 3 * z )`
- Custom expressions

```cpp
CP::Expression objective(3 * x + 5 * y);
```

### Constraints

Any expression can be added to the model as a constraint.

```cpp
model.addConstraint(x + y <= 10);
```

### Objective

The objective can be specified by providing the objective sense and an expression for the objective function.

```cpp
CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
model.setObjective(3 * x + 5 * y);
```

### Solution

Stores a concrete assignment to variables for a given model and allows evaluation of expressions.

```cpp
CP::Solution solution(model);
solution.setVariableValue(x, 3);
solutionsolution.setVariableValue(y, 4);
auto result = solution.evaluate(3 * x + 5 * y); // should return 29
```

## Example usage

The `main.cpp` file contains a comprehensive collection of examples.

## Building

This is a header-only library. To use it, simply include the header in your project.

To build and run tests:

```
make clean; make; ./test
```

Build and run tests with limex (change path as necessary):

```
make clean; make LIMEX_PATH=../limex; ./test
```


## License

MIT License

Copyright (c) 2025 Asvin Goel
