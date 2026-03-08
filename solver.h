#pragma once

#include "cp.h"
#include <string>
#include <memory>
#include <functional>
#include <atomic>
#include <limits>

namespace CP {

/**
 * @brief Abstract base class for optimization solvers.
 *
 * Provides a common interface for different solver backends (e.g., SCIP, Hexaly).
 * Supports warmstarting, callbacks for solution/iteration events, and interruption.
 *
 * @note Current solution is preserved across solve() calls and updated with the best
 * solution found during solving. Call setSolution(nullptr) for a fresh start.
 */
class Solver {
public:
  /**
   * @brief Constructs a solver for the given model.
   * @param model The optimization model to solve. Must outlive the Solver.
   */
  explicit Solver(const Model& model) : model_(model) {}

  virtual ~Solver() = default;

  Solver(const Solver&) = delete;
  Solver& operator=(const Solver&) = delete;
  Solver(Solver&&) = delete;
  Solver& operator=(Solver&&) = delete;

  /**
   * @brief Returns the name of the solver backend.
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Result status returned by solve().
   */
  struct Result {
    /**
     * @brief Problem feasibility status.
     */
    enum class PROBLEM {
      FEASIBLE,   ///< Problem has at least one feasible solution.
      INFEASIBLE, ///< Problem is infeasible.
      UNBOUNDED,  ///< Problem objective is unbounded.
      UNKNOWN     ///< Feasibility could not be determined.
    };

    /**
     * @brief Solution quality status.
     */
    enum class SOLUTION {
      NONE,     ///< No solution found.
      FEASIBLE, ///< Solution is feasible, optimality is not proven.
      OPTIMAL   ///< Solution is provenly optimal.
    };

    /**
     * @brief Reason for solver termination.
     */
    enum class TERMINATION {
      TIMEOUT,     ///< Terminated due to time limit.
      COMPLETED,   ///< Terminated normally (optimality/infeasibility/unboundedness proven).
      INTERRUPTED, ///< Terminated by user via stop().
      OTHER        ///< Terminated for another reason.
    };

    PROBLEM problem = PROBLEM::UNKNOWN;
    SOLUTION status = SOLUTION::NONE;
    TERMINATION termination = TERMINATION::OTHER;
    std::string info; ///< Optional solver feedback/diagnostics.
  };

  /// Callback invoked when a new best solution is found.
  using SolutionListener = std::function<void(const Solution&)>;

  /// Callback invoked at each solver iteration.
  using IterationListener = std::function<void()>;

  /**
   * @brief Registers a callback for new best solutions.
   * @param callback Function to call when a new best solution is found. Pass nullptr to clear.
   * @note Overwrites any previously registered solution listener.
   */
  void registerListener(SolutionListener callback) { onSolution = std::move(callback); }

  /**
   * @brief Registers a callback for solver iterations.
   * @param callback Function to call at each iteration. Pass nullptr to clear.
   * @note Overwrites any previously registered iteration listener.
   */
  void registerListener(IterationListener callback) { onIteration = std::move(callback); }

  /**
   * @brief Returns the current best solution (thread-safe).
   * @return Shared pointer to the solution, or nullptr if none found.
   */
  std::shared_ptr<const Solution> getSolution() const {
    return solution_.load();
  }

  /**
   * @brief Sets an initial solution for warmstarting (thread-safe).
   * @param solution The solution to use as a starting point.
   */
  void setSolution(std::shared_ptr<const Solution> solution) {
    solution_.store(std::move(solution));
  }

  /**
   * @brief Solves the model.
   * @param timeLimit Maximum solving time in seconds (default: no limit).
   * @return Result status indicating problem feasibility, solution quality, and termination reason.
   */
  Result solve(double timeLimit = std::numeric_limits<double>::infinity()) {
    interrupted_ = false;
    return solve_(timeLimit);
  }

  /**
   * @brief Requests the solver to stop as soon as possible.
   *
   * Can be called from callbacks. The solve() method will return with
   * termination == INTERRUPTED.
   */
  void stop() { interrupted_ = true; }

protected:
  /**
   * @brief Implementation of solve(), to be overridden by derived classes.
   * @param timeLimit Maximum solving time in seconds.
   * @return Result status.
   */
  virtual Result solve_(double timeLimit) = 0;

  const Model& model_;                       ///< The model being solved.
  mutable std::atomic<std::shared_ptr<const Solution>> solution_; ///< Current best / warmstart solution (thread-safe).
  std::atomic<bool> interrupted_{false};     ///< Interruption flag.
  SolutionListener onSolution;               ///< Solution callback.
  IterationListener onIteration;             ///< Iteration callback.
};

} // namespace CP
