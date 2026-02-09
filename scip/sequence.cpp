#include "sequence.h"
#include <scip/scip.h>
#include <scip/cons_linear.h>
#include <vector>
#include <string>

namespace CP {

void addSequenceConstraints(SCIP* scip,
                            const std::string& seqName,
                            const std::vector<SCIP_VAR*>& seqVars,
                            int minVal,
                            int maxVal) {
    size_t n = seqVars.size();

    // Binary matrix formulation for alldifferent
    // Create n×(maxVal-minVal+1) binary variables b[i][v] for each position i and value v
    std::vector<std::vector<SCIP_VAR*>> binaries(n);
    for (size_t i = 0; i < n; i++) {
        binaries[i].resize(maxVal - minVal + 1);
        for (int v = minVal; v <= maxVal; v++) {
            std::string binName = seqName + "_b[" + std::to_string(i) + "][" + std::to_string(v) + "]";
            SCIP_VAR* binVar;
            SCIPcreateVarBasic(scip, &binVar, binName.c_str(),
                              0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
            SCIPaddVar(scip, binVar);
            binaries[i][v - minVal] = binVar;
        }
    }

    // Row constraints: sum_v b[i][v] = 1 for each position i
    for (size_t i = 0; i < n; i++) {
        SCIP_CONS* rowCons;
        std::vector<double> coeffs(binaries[i].size(), 1.0);
        std::string consName = seqName + "_row[" + std::to_string(i) + "]";
        SCIPcreateConsBasicLinear(scip, &rowCons, consName.c_str(),
                                 binaries[i].size(), binaries[i].data(), coeffs.data(),
                                 1.0, 1.0);
        SCIPaddCons(scip, rowCons);
        SCIPreleaseCons(scip, &rowCons);
    }

    // Column constraints: sum_i b[i][v] = 1 for each value v
    for (int v = minVal; v <= maxVal; v++) {
        SCIP_CONS* colCons;
        std::vector<SCIP_VAR*> colVars(n);
        std::vector<double> coeffs(n, 1.0);
        for (size_t i = 0; i < n; i++) {
            colVars[i] = binaries[i][v - minVal];
        }
        std::string consName = seqName + "_col[" + std::to_string(v) + "]";
        SCIPcreateConsBasicLinear(scip, &colCons, consName.c_str(),
                                 n, colVars.data(), coeffs.data(),
                                 1.0, 1.0);
        SCIPaddCons(scip, colCons);
        SCIPreleaseCons(scip, &colCons);
    }

    // Link constraints: x[i] = sum_v (v * b[i][v])
    for (size_t i = 0; i < n; i++) {
        SCIP_VAR* scipVar = seqVars[i];

        SCIP_CONS* linkCons;
        std::vector<SCIP_VAR*> linkVars;
        std::vector<double> linkCoeffs;

        // Add x[i] with coefficient -1
        linkVars.push_back(scipVar);
        linkCoeffs.push_back(-1.0);

        // Add b[i][v] with coefficient v
        for (int v = minVal; v <= maxVal; v++) {
            linkVars.push_back(binaries[i][v - minVal]);
            linkCoeffs.push_back(static_cast<double>(v));
        }

        std::string consName = seqName + "_link[" + std::to_string(i) + "]";
        SCIPcreateConsBasicLinear(scip, &linkCons, consName.c_str(),
                                 linkVars.size(), linkVars.data(), linkCoeffs.data(),
                                 0.0, 0.0);
        SCIPaddCons(scip, linkCons);
        SCIPreleaseCons(scip, &linkCons);
    }

    // Release binary variables
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < binaries[i].size(); j++) {
            SCIPreleaseVar(scip, &binaries[i][j]);
        }
    }
}

} // namespace CP
