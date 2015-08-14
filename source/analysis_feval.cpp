/* ===============================================================
 * Analysis of feval instructions in the code.
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */
#include "analysis_feval.h"
#include "functions.h"
#include "stmtsequence.h"
#include "typeinfer.h"
#include "paramexpr.h"
#include "exprstmt.h"
#include "assignstmt.h"
#include "ifelsestmt.h"
#include "loopstmts.h"

#include <set>

inline bool checkIfIsFeval(ParamExpr* pExpr) {
    return (pExpr->getSymExpr()->getSymName() == "feval");
}

void parseStatementForFevalCalls(Statement* stmt, std::set<AssignStmt*> &fevalInstructions) {
    switch (stmt->getStmtType()) {
            case Statement::ASSIGN: {
                AssignStmt* pAssignStmt = (AssignStmt*)stmt;
                Expression* pRightExpr = pAssignStmt->getRightExpr();
                if (pRightExpr->getExprType() == Expression::PARAM) {
                    ParamExpr* pParamExpr = (ParamExpr*)pRightExpr;
                    if (checkIfIsFeval(pParamExpr)) {
                        fevalInstructions.insert(pAssignStmt);
                    }
                }

            }
            break;

            case Statement::BREAK:
            case Statement::CONTINUE:
            case Statement::RETURN:
            {
                // nothing to do!
            }
            break;

            case Statement::COMPOUND_END: {
                assert(false);
            }
            break;

            case Statement::IF_ELSE: {
                IfElseStmt* pIfElseStmt = (IfElseStmt*)stmt;

                for (Statement* pStmt: pIfElseStmt->getIfBlock()->getStatements()) {
                    parseStatementForFevalCalls(pStmt, fevalInstructions);
                }

                for (Statement* pStmt: pIfElseStmt->getElseBlock()->getStatements()) {
                    parseStatementForFevalCalls(pStmt, fevalInstructions);
                }
            }
            break;

            case Statement::EXPR: {
                /* TODO: might it be worth optimizing these ones to
                 *       enable better type inference?
                ExprStmt* pExprStmt = (ExprStmt*)stmt;
                Expression* pExpr = pExprStmt->getExpression();
                if (pExpr->getExprType() == Expression::PARAM) {
                    ParamExpr* pParamExpr = (ParamExpr*)pExpr;
                    if (checkIfIsFeval(pParamExpr)) {
                        std::cout << "feval function found!" << std::endl;
                        fevalInstructions.insert(pParamExpr);
                    }
                }
                */
            }
            break;

            case Statement::LOOP: { // handles for, loop, while
                LoopStmt* pLoopStmt =(LoopStmt*)stmt;

                // for the time being we optimize loop body only
                for (Statement* pStmt: pLoopStmt->getBodySeq()->getStatements()) {
                    parseStatementForFevalCalls(pStmt, fevalInstructions);
                }
            }
            break;

            default: {
                assert(false);
            }

        }
}

AnalysisInfo* computeFevalInfo(
	const ProgFunction* pFunction,
	const StmtSequence* pFuncBody,
	const TypeSetString& inArgTypes,
	bool returnBottom
) {
    // Create a reaching definition info object
    FevalInfo* pFevalInfo = new FevalInfo();

    // If we should return bottom, return no information
    if (returnBottom) {
        return pFevalInfo;
    }

    std::cerr << "--> feval Analysis <--" << std::endl;

    const ProgFunction::ParamVector& inParams = pFunction->getInParams();

    std::set<const SymbolExpr*> readOnlyParams;
    std::set<AssignStmt*> fevalInstructions;

    for (const SymbolExpr* sym: inParams) {
        std::cout << "Parameter: " << sym->getSymName() << std::endl;
        readOnlyParams.insert(sym);
    }

    // Get a reference to the statement vector
    const StmtSequence::StmtVector& stmts = pFuncBody->getStatements();

    // For each statement
    size_t numStmts = stmts.size();
    for (size_t i = 0; i < numStmts; ++i) {

        // Get a pointer to the statement
	Statement* pStmt = const_cast<Statement*>(stmts[i]);

        //std::cerr << "[" << pStmt->getAnnotations() << "] " << pStmt->toString() << std::endl;

        parseStatementForFevalCalls(pStmt, fevalInstructions);

    }

    if (fevalInstructions.empty()) {
        std::cerr << "No feval instruction found." << std::endl;
        return pFevalInfo;
    } else {
        std::cerr << "=======================" << std::endl;
        std::cerr << "Found feval statements:" << std::endl;
        for (AssignStmt* pAssignStmt: fevalInstructions) {
            std::cerr << pAssignStmt->toString() << std::endl;
        }
        std::cerr << "=======================" << std::endl;
    }



    return pFevalInfo;
}