#pragma once

#include "cp.h"
#include <string>
#include <expected>

namespace CP {

/**
 * @brief Abstract base class for solvers
 */
class Solver {
public:
    virtual ~Solver() = default;
    virtual std::expected<Solution, std::string> solve(const Model& model) = 0;
    virtual std::string getName() const = 0;
};

} // namespace CP
