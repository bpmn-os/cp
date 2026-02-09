#pragma once

#include <scip/scip.h>
#include <vector>
#include <string>

namespace CP {

/**
 * @brief Adds sequence constraint (alldifferent permutation with domain [minVal, maxVal])
 *
 * Uses binary matrix formulation with binary variables b[i][v] for each position i and value v:
 * - sum_v b[i][v] = 1 for each position (each position gets one value)
 * - sum_i b[i][v] = 1 for each value (each value used once)
 * - x[i] = sum_v (v * b[i][v]) (link binary to sequence variable)
 *
 * This enforces that the sequence variables form a permutation of [minVal, maxVal].
 *
 * @param scip SCIP instance
 * @param seqName Name of the sequence (for constraint naming)
 * @param seqVars Sequence variables
 * @param minVal Minimum value in the domain
 * @param maxVal Maximum value in the domain
 */
void addSequenceConstraints(SCIP* scip,
                            const std::string& seqName,
                            const std::vector<SCIP_VAR*>& seqVars,
                            int minVal,
                            int maxVal);

} // namespace CP
