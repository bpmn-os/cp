#pragma once

#include <scip/scip.h>
#include <vector>
#include <string>

namespace CP {

/**
 * @brief Adds element constraint: result = array[index]
 *
 * Uses binary decomposition:
 * - For each position i: binary variable b[i]
 * - sum(b[i]) = 1 (exactly one position selected)
 * - index = sum((i+offset) * b[i]) (link binary to index)
 * - result = sum(array[i] * b[i]) (link binary to result)
 *
 * @param scip SCIP instance
 * @param name Name for constraint naming
 * @param arrayVars Array of SCIP variables
 * @param indexVar Index variable (determines which array element)
 * @param resultVar Result variable (will equal array[index])
 * @param indexOffset Offset for index (e.g., if index starts at 1, offset is 1)
 * @return SCIP expression for the result
 */
SCIP_EXPR* addElementConstraint(SCIP* scip,
                                 const std::string& name,
                                 const std::vector<SCIP_VAR*>& arrayVars,
                                 SCIP_VAR* indexVar,
                                 SCIP_VAR* resultVar,
                                 int indexOffset);

} // namespace CP
