#include "indexed_variables.h"
#include <scip/scip.h>
#include <scip/cons_linear.h>
#include <scip/cons_nonlinear.h>
#include <scip/expr_var.h>
#include <scip/expr_product.h>
#include <scip/expr_sum.h>
#include <vector>
#include <string>

namespace CP {

SCIP_EXPR* addElementConstraint(SCIP* scip,
                                 const std::string& name,
                                 const std::vector<SCIP_VAR*>& arrayVars,
                                 SCIP_VAR* indexVar,
                                 SCIP_VAR* resultVar,
                                 int indexOffset) {
    size_t n = arrayVars.size();

    // Create binary variables b[i] for each position i
    std::vector<SCIP_VAR*> binaries(n);
    for (size_t i = 0; i < n; i++) {
        std::string binName = name + "_b[" + std::to_string(i) + "]";
        SCIP_VAR* binVar;
        SCIPcreateVarBasic(scip, &binVar, binName.c_str(),
                          0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY);
        SCIPaddVar(scip, binVar);
        binaries[i] = binVar;
    }

    // Constraint: sum(b[i]) = 1 (exactly one position selected)
    SCIP_CONS* sumCons;
    std::vector<double> ones(n, 1.0);
    std::string sumName = name + "_sum";
    SCIPcreateConsBasicLinear(scip, &sumCons, sumName.c_str(),
                             n, binaries.data(), ones.data(),
                             1.0, 1.0);
    SCIPaddCons(scip, sumCons);
    SCIPreleaseCons(scip, &sumCons);

    // Constraint: index = sum((i+offset) * b[i])
    SCIP_CONS* indexCons;
    std::vector<SCIP_VAR*> indexVars;
    std::vector<double> indexCoeffs;

    indexVars.push_back(indexVar);
    indexCoeffs.push_back(-1.0);

    for (size_t i = 0; i < n; i++) {
        indexVars.push_back(binaries[i]);
        indexCoeffs.push_back(static_cast<double>(i + indexOffset));
    }

    std::string indexName = name + "_index";
    SCIPcreateConsBasicLinear(scip, &indexCons, indexName.c_str(),
                             indexVars.size(), indexVars.data(), indexCoeffs.data(),
                             0.0, 0.0);
    SCIPaddCons(scip, indexCons);
    SCIPreleaseCons(scip, &indexCons);

    // Constraint: result = sum(b[i] * array[i])
    // This is algebraic (no big-M), consistent with if-then-else implementation
    std::vector<SCIP_EXPR*> productExprs;

    for (size_t i = 0; i < n; i++) {
        // Create b[i] * array[i]
        SCIP_EXPR* binExpr;
        SCIPcreateExprVar(scip, &binExpr, binaries[i], nullptr, nullptr);

        SCIP_EXPR* arrayExpr;
        SCIPcreateExprVar(scip, &arrayExpr, arrayVars[i], nullptr, nullptr);

        SCIP_EXPR* productExpr;
        SCIP_EXPR* factors[] = { binExpr, arrayExpr };
        SCIPcreateExprProduct(scip, &productExpr, 2, factors, 1.0, nullptr, nullptr);

        productExprs.push_back(productExpr);

        SCIPreleaseExpr(scip, &binExpr);
        SCIPreleaseExpr(scip, &arrayExpr);
    }

    // Create sum of products
    SCIP_EXPR* sumExpr;
    std::vector<double> coeffs(productExprs.size(), 1.0);
    SCIPcreateExprSum(scip, &sumExpr, productExprs.size(), productExprs.data(),
                      coeffs.data(), 0.0, nullptr, nullptr);

    // Create result variable expression
    SCIP_EXPR* resultExpr_tmp;
    SCIPcreateExprVar(scip, &resultExpr_tmp, resultVar, nullptr, nullptr);

    // Constraint: result - sum(b[i] * array[i]) = 0
    SCIP_EXPR* diffExpr;
    SCIP_EXPR* diffChildren[] = { resultExpr_tmp, sumExpr };
    double diffCoeffs[] = { 1.0, -1.0 };
    SCIPcreateExprSum(scip, &diffExpr, 2, diffChildren, diffCoeffs, 0.0, nullptr, nullptr);

    SCIP_CONS* resultCons;
    std::string resultConsName = name + "_result";
    SCIPcreateConsBasicNonlinear(scip, &resultCons, resultConsName.c_str(),
                                 diffExpr, 0.0, 0.0);
    SCIPaddCons(scip, resultCons);
    SCIPreleaseCons(scip, &resultCons);

    // Release expressions
    SCIPreleaseExpr(scip, &diffExpr);
    SCIPreleaseExpr(scip, &resultExpr_tmp);
    SCIPreleaseExpr(scip, &sumExpr);
    for (auto expr : productExprs) {
        SCIPreleaseExpr(scip, &expr);
    }

    // Release binary variables
    for (size_t i = 0; i < n; i++) {
        SCIPreleaseVar(scip, &binaries[i]);
    }

    // Return expression for result variable
    SCIP_EXPR* resultExpr;
    SCIPcreateExprVar(scip, &resultExpr, resultVar, nullptr, nullptr);
    return resultExpr;
}

} // namespace CP
