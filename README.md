# Constraint Programming Interface (CP)

A C++ constraint programming library for modeling constraint programming problems. It provides constructs for variables, expressions, sequences, constraints, and solutions, enabling the construction and manipulation of constraint models in an intuitive way.

## Architecture

- **cp.h** - Header-only CP modeling interface (no dependencies)
- **solver.h** - Abstract solver interface
- **scip/** - SCIP solver adapter (requires SCIP, see below)
- **hexaly/** - Hexaly solver adapter (requires Hexaly, see below)
- **limex_handle.h** - Custom operator support via [LIMEX](https://github.com/bpmn-os/limex)

## Features

- Boolean, integer, and real variables
- Indexed variables for array-like structures
- Collections with indexed access and aggregate operations (count, sum, avg, min, max)
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
auto& y = model.addIntegerVariable("y");  // y ∈ { -infinity, ..., infinity }
auto& z = model.addBinaryVariable("z");   // z ∈ { false, true }
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
// create container with item type and name
auto& a = model.addIndexedVariables(CP::Variable::Type::INTEGER, "a");
// create each item with bounds or expression
model.addIndexedVariable( a, 0, 5 );      // a[0] ∈ { 0, ..., 5 }
model.addIndexedVariable( a, x + 4 );     // a[1] := x + 4.00 ( x must have been added to the model before )
model.addIndexedVariable( a, a[1] + 5 );  // a[2] := a[1] + 5.00 ( only variables with lower index must be used )
```

### Collections

The `Collection` struct provides access to static external data collections that can be looked up at runtime. A collection is identified by a key (variable or constant) and supports indexed access and aggregate operations.

```cpp
std::vector<std::vector<double>> collections = {
  {10.0, 20.0, 30.0},
  {40.0, 50.0}
};

// Set up collection lookup function (caller responsible for bounds)
model.setCollectionLookup([&collections](size_t key) -> const std::vector<double>& {
  return collections[key];
}, collections.size());  // 2 collections

auto& key = model.addVariable(CP::Variable::Type::INTEGER, "key", 0, 1);
auto collection = CP::Collection(key);

// Indexed access
auto x1 = collection[1];       // first element of collection identified by key (1-based indexing)
auto& index = model.addVariable(CP::Variable::Type::INTEGER, "index", 1, 2);
auto x2 = collection[index];   // element at variable index

// Aggregate operations
auto y1 = CP::count(collection);   // number of elements
auto y2 = CP::sum(collection);     // sum of elements
auto y3 = CP::avg(collection);     // average of elements
auto y4 = CP::max(collection);     // maximum element
auto y5 = CP::min(collection);     // minimum element

// Membership operations
auto& value = model.addVariable(CP::Variable::Type::REAL, "value", 0.0, 100.0);
auto z1 = CP::element_of(value, collection);       // true if value is in collection
auto z2 = CP::not_element_of(value, collection);   // true if value is not in collection
```

### Sequences

The struct `Sequence` represents a permutation of variables with a given size.

```cpp
auto& s = model.addSequence("s", 3 ); // ( s[0], s[1], s[2] ) is permutation of { 1, ..., 3 }
```


### Expressions

Expressions can be intuitively written using the variable references previously added to a model.

- Arithmetic expressions using operators `+`, `-`, `*`, `/`, e.g. `x + 3 * y`
- Logical expression using operators `&&`, `||`, `!`, e.g. `x && ( y || !z )`
- Indexing using `[` and `]`, e.g. `a[i]`
- Comparisons using operators `<`, `>`, `<=`, `>=`, `==`, `!=`, e.g. `x < a[i]`
- Implications, e.g. `x.implies(y < 5)`
- Min/max expressions, e.g `CP::min( 0, x, 3 * z )`, `CP::max( 0.0, x, 3 * z )`
- Conditional expressions:
  - `CP::if_then_else( condition, thenExpr, elseExpr )`
  - `CP::n_ary_if( {{condition_1, expression_1}, {condition_2, expression_2}, ...}, expression_n )` (assumes that at most one condition_i is true)
- Custom expressions, e.g. those provided by LIMEX (see below)

```cpp
CP::Expression objective(3 * x + 5 * y);
```

### Constraints

Any expression can be added to the model as a constraint.

```cpp
model.addConstraint(x + y <= 10);
```

### Objective

The objective can be specified by providing the objective sense `CP::Model::ObjectiveSense::FEASIBLE` (default), `CP::Model::ObjectiveSense::MAXIMIZE`, or `CP::Model::ObjectiveSense::MINIMIZE` and an expression for the objective function.

```cpp
CP::Model model(CP::Model::ObjectiveSense::MINIMIZE);
auto& x = model.addRealVariable("x");     // x ∈ [ -infinity, infinity ]
auto& y = model.addIntegerVariable("y");  // y ∈ { -infinity, ..., infinity }
model.setObjective(3 * x + 5 * y);
```

### Solution

Stores a concrete assignment to variables for a given model and allows evaluation of expressions and constraints. Deduced variable values are cached for performance - repeated evaluations return the cached result. The cache is automatically invalidated when a dependency changes via `setVariableValue()`.

```cpp
CP::Solution solution(model);
solution.setVariableValue(x, 3);
solution.setVariableValue(y, 4);
auto result = solution.evaluate(3 * x + 5 * y == 29); // evaluates to 1 (true)
```

### Solver

The `Solver` class provides an abstract interface for optimization solvers. Concrete implementations include `SCIPSolver` and `HexalySolver`.

#### Basic solving

```cpp
CP::SCIPSolver solver(model);
auto result = solver.solve();  // Solve without time limit

if (result.status != CP::Solver::Result::SOLUTION::NONE) {
    auto solution = solver.getSolution();
    std::cout << solution->stringify() << std::endl;
}
```

#### Solving with time limit

```cpp
auto result = solver.solve(60.0);  // 60 second time limit

if (result.termination == CP::Solver::Result::TERMINATION::TIMEOUT) {
    std::cout << "Time limit reached" << std::endl;
}
```

#### Warmstart

By default, `solve()` uses the current solution (from a previous solve or `setSolution()`) as a starting point. You can provide an initial solution to speed up solving:

```cpp
// Create and set initial solution
auto initialSolution = std::make_shared<CP::Solution>(model);
initialSolution->setSequenceValues(sequence, std::vector<int>{2, 1, 3, 4});
initialSolution->setVariableValue(x, 10.0);
initialSolution->setVariableValue(y, 20.0);

solver.setSolution(initialSolution);
auto result = solver.solve();  // Solver starts from initial solution
```

#### Cold start

To start fresh without a previous solution:

```cpp
solver.setSolution(nullptr);  // Clear any previous solution
auto result = solver.solve();
```

#### Solution callback

Register a callback to be notified when a new best solution is found:

```cpp
solver.registerListener(CP::Solver::SolutionListener([](const CP::Solution& solution) {
    std::cout << "New solution found: " << solution.stringify() << std::endl;
}));

auto result = solver.solve();
```

#### Iteration callback

Register a callback to be notified at each solver iteration:

```cpp
int iterations = 0;
solver.registerListener(CP::Solver::IterationListener([&iterations]() {
    iterations++;
}));

auto result = solver.solve();
std::cout << "Iterations: " << iterations << std::endl;
```

#### Stopping the solver

Stop the solver from a callback:

```cpp
solver.registerListener(CP::Solver::IterationListener([&solver]() {
    if (/* some condition */) {
        solver.stop();  // Terminates solving
    }
}));

auto result = solver.solve();
if (result.termination == CP::Solver::Result::TERMINATION::INTERRUPTED) {
    std::cout << "Solver was stopped" << std::endl;
}
```

#### Result status

The `Result` struct provides information about the solving process:

```cpp
auto result = solver.solve();

// Problem feasibility
result.problem;  // FEASIBLE, INFEASIBLE, UNBOUNDED, UNKNOWN

// Solution quality
result.status;  // NONE, FEASIBLE, OPTIMAL

// Termination reason
result.termination;  // TIMEOUT, COMPLETED, INTERRUPTED, OTHER

// Optional info message
result.info;
```

### LIMEX

[LIMEX](https://github.com/bpmn-os/limex) is a C++ library for passing mathematical expressions. The library can be used to add user provided expressions to a constraint program allowing custom operators like `abs`, `pow`, `sqrt`, `cbrt`, `sum`, `avg`, `count`, `∈`, `∉`. More details can be found in the file `limex_handle.cpp` and in the [LIMEX](https://github.com/bpmn-os/limex) repository.

## Example usage

The `main.cpp` file contains a comprehensive collection of examples.

## Building

### Header-only modeling (no solver)

Simply include `cp.h` in your project - no build or linking required.

```cpp
#include "cp.h"

CP::Model model;
auto& x = model.addIntegerVariable("x");
model.addConstraint(x >= 5);
std::cout << model.stringify() << std::endl;
```

### With SCIP solver

**Prerequisites:** Install SCIP 10.0.1+ from [scipopt.org](https://scipopt.org)

**Build:**
```bash
mkdir build && cd build
cmake ..
make
```

This builds:
- `libcp-scip.a` / `libcp-scip.so` - SCIP adapter library
- `test_scip` - Test suite

**CMake options:**
```bash
cmake .. -DBUILD_TESTS=OFF     # Skip tests
```

### Using the SCIP adapter

**Code:**
```cpp
#include "cp.h"
#include "scip/scip_adapter.h"

CP::Model model;
auto& x = model.addIntegerVariable("x");
model.addConstraint(x >= 5);

CP::SCIPSolver solver(model);
auto result = solver.solve();

if (result.status != CP::Solver::Result::SOLUTION::NONE) {
    std::cout << solver.getSolution()->stringify() << std::endl;
}
```

**CMakeLists.txt:**
```cmake
find_package(cp REQUIRED)
find_package(cp-scip REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE cp::cp cp::scip)
```

### With Hexaly solver

**Prerequisites:** Install Hexaly from [hexaly.com](https://www.hexaly.com)

**Build:**
```bash
mkdir build && cd build
cmake ..
make
```

This builds:
- `libcp-hexaly.a` / `libcp-hexaly.so` - Hexaly adapter library
- `test_hexaly` - Test suite

### Using the Hexaly adapter

**Code:**
```cpp
#include "cp.h"
#include "hexaly/hexaly_adapter.h"

CP::Model model;
auto& x = model.addIntegerVariable("x");
model.addConstraint(x >= 5);

CP::HexalySolver solver(model);
auto result = solver.solve(60.0);  // 60 second time limit

if (result.status != CP::Solver::Result::SOLUTION::NONE) {
    std::cout << solver.getSolution()->stringify() << std::endl;
}
```

**CMakeLists.txt:**
```cmake
find_package(cp REQUIRED)
find_package(cp-hexaly REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE cp::cp cp::hexaly)
```


## License

MIT License

Copyright (c) 2025 Asvin Goel
