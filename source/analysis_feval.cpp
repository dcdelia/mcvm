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
#include "configmanager.h"

#include <map>
#include <utility>

void parseStatementForFevalCalls(Statement* stmt, StmtSequence* parentStmtSeq, FevalInfo* analysisInfo, int loop_depth) {
    switch (stmt->getStmtType()) {
        case Statement::ASSIGN:
        {
            AssignStmt* pAssignStmt = (AssignStmt*) stmt;
            Expression* pRightExpr = pAssignStmt->getRightExpr();
            if (pRightExpr->getExprType() == Expression::PARAM) {
                ParamExpr* pParamExpr = (ParamExpr*) pRightExpr;
                if (pParamExpr->getSymExpr()->getSymName() == "feval") {
                    FevalInfo::FevalCallInfo *callInfo = new FevalInfo::FevalCallInfo();

                    callInfo->assStmt = pAssignStmt;
                    callInfo->pExpr = pParamExpr;
                    callInfo->loopIdx = loop_depth; // TODO do we need to distinguish loops?
                    callInfo->parentStmtSeq = parentStmtSeq;
                    callInfo->firstArg = pParamExpr->getArgument(0);

                    // it is important to preserve insertion order
                    analysisInfo->FevalCalls.push_back(callInfo);
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

        case Statement::COMPOUND_END:
        {
            assert(false);
        }
            break;

        case Statement::IF_ELSE:
        {
            IfElseStmt* pIfElseStmt = (IfElseStmt*) stmt;

            StmtSequence* seq = pIfElseStmt->getIfBlock();
            analysisInfo->ParentMap[seq] = parentStmtSeq;
            for (Statement* pStmt : seq->getStatements()) {
                parseStatementForFevalCalls(pStmt, seq, analysisInfo, loop_depth);
            }

            seq = pIfElseStmt->getElseBlock();
            analysisInfo->ParentMap[seq] = parentStmtSeq;
            for (Statement* pStmt : seq->getStatements()) {
                parseStatementForFevalCalls(pStmt, seq, analysisInfo, loop_depth);
            }
        }
            break;

        case Statement::EXPR:
        {
            /* TODO: optimizing these ones might enable better type inference? */
        }
            break;

        case Statement::LOOP:
        { // handles for, loop, and while since IIR has been transformed
            LoopStmt* pLoopStmt = (LoopStmt*) stmt;

            StmtSequence* seq = pLoopStmt->getBodySeq();
            analysisInfo->ParentMap[seq] = parentStmtSeq;
            // for the time being we optimize loop body only
            for (Statement* pStmt : seq->getStatements()) {
                parseStatementForFevalCalls(pStmt, seq, analysisInfo, loop_depth + 1);
            }
        }
            break;

        default:
        {
            assert(false);
        }

    }
}

std::map<FevalInfo::FunMapKey, FevalInfo*> FevalInfo::FevalInfoMap;

AnalysisInfo* computeFevalInfo(const ProgFunction* pFunction, const StmtSequence* pFuncBody,
        const TypeSetString& inArgTypes, bool returnBottom) {
    // If we should return bottom, return no information
    if (returnBottom) {
        return new FevalInfo();
    }

    FevalInfo::FunMapKey key(pFunction, pFuncBody); // analysis does not depend on inArgTypes

    std::map<FevalInfo::FunMapKey, FevalInfo*>::iterator it = FevalInfo::FevalInfoMap.find(key);
    if (it != FevalInfo::FevalInfoMap.end()) {
        return it->second;
    }

    FevalInfo* pFevalInfo = new FevalInfo();
    FevalInfo::FevalInfoMap.insert(std::pair<FevalInfo::FunMapKey, FevalInfo*>(key, pFevalInfo));

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "feval analysis for function " << pFunction->getFuncName() << std::endl;
    }

    pFevalInfo->ParentMap[const_cast<StmtSequence*>(pFuncBody)] = nullptr;

    // Get a reference to the statement vector
    const StmtSequence::StmtVector& stmts = pFuncBody->getStatements();

    // For each statement
    size_t numStmts = stmts.size();
    for (size_t i = 0; i < numStmts; ++i) {

        // Get a pointer to the statement
        Statement* pStmt = const_cast<Statement*> (stmts[i]);

        parseStatementForFevalCalls(pStmt, nullptr, pFevalInfo, 0); // nullptr acts as root

    }

    if (pFevalInfo->FevalCalls.empty()) {
        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
            std::cerr << "No feval instruction found." << std::endl;
        }
        return pFevalInfo;
    }
    pFevalInfo->containsFevalInstructions = true;

    /* Determine if there are feval statements worth the optimization
     *
     * Heuristic in use: at least one of the found feval instructions must be inside a loop
     */
    bool optimize = false;
    for (FevalInfo::FevalCallInfo* callInfo : pFevalInfo->FevalCalls) {
        optimize |= (callInfo->loopIdx > 0);
    }
    if (!optimize) return pFevalInfo;

    /* Compute reaching definition information and group feval statements
     * that are reached by the same definitions. */
    ReachDefInfo* pReachDefInfo = (ReachDefInfo*) AnalysisManager::requestInfo(
            &computeReachDefs, pFunction, pFuncBody, inArgTypes);

    const ProgFunction::ParamVector& inParams = pFunction->getInParams();

    // it is important to preserve insertion order in the scan
    for (FevalInfo::FevalCallInfo* callInfo: pFevalInfo->FevalCalls) {

        ReachDefMap::const_iterator defItr = pReachDefInfo->reachDefMap.find(callInfo->assStmt);
        assert(defItr != pReachDefInfo->reachDefMap.end());

        const VarDefMap& reachDefs = defItr->second;

        Expression::ExprType exprType = callInfo->firstArg->getExprType();
        if (exprType == Expression::SYMBOL) {
            SymbolExpr *symExpr = (SymbolExpr*) callInfo->firstArg;
            if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
                std::cerr << "Analyzing symbol argument " << symExpr->toString()
                    << " for feval instruction " << callInfo->pExpr->toString() << std::endl;
            }

            VarDefMap::const_iterator symDefItr = reachDefs.find(symExpr);
            assert(symDefItr != reachDefs.end());

            const VarDefSet &defSet = symDefItr->second;

            /* when the argument is a read-only function parameter I get nullptr
               as first element and the whole function as second element */
            if (defSet.size() == 2) {
                const IIRNode* first = *(defSet.begin());
                const IIRNode* second = *++(defSet.begin());
                if (first == nullptr && second == (const IIRNode*) pFunction) {
                    SymbolExpr* funArg = nullptr;
                    for (const SymbolExpr* arg : inParams) {
                        if (arg->toString() == symExpr->toString()) {
                            funArg = const_cast<SymbolExpr*> (arg);
                            break;
                        }
                    }
                    assert(funArg != nullptr);

                    FevalInfo::SymToStatementsMap &cfaMap = pFevalInfo->ConstantFirstArg;
                    FevalInfo::SymToStatementsMap::iterator cfaIt = cfaMap.find(funArg);
                    if (cfaIt == cfaMap.end()) {
                        cfaMap.insert(std::pair<SymbolExpr*, std::vector < FevalInfo::FevalCallInfo*>>(funArg,{callInfo}));
                    } else {
                        // it is important to preserve insertion order
                        cfaIt->second.push_back(callInfo);
                    }

                    continue; // process next feval call
                }
            }

            std::cerr << "Sorry, I don't know how to optimize this feval call yet! --> ";
            std::cerr << callInfo->assStmt->toString() << std::endl;
            std::cerr << "defSet size: " << defSet.size() << std::endl;
        } else {
            if (exprType == Expression::STR_CONST) {
                if (ConfigManager::s_veryVerboseVar || ConfigManager::s_verboseVar) {
                    std::cerr << "Ignoring this feval call as its first argument is a constant string! --> ";
                    std::cerr << callInfo->assStmt->toString() << std::endl;
                }
            } else if (exprType == Expression::FN_HANDLE) {
                std::cerr << "Sorry, I don't know how to treat function handles yet! --> ";
                std::cerr << callInfo->assStmt->toString() << std::endl;
            } else {
                std::cerr << "Found an Expression of unexpected type " << exprType << " when analyzing a feval!!!!" << std::endl;
                assert(false);
            }
        }
    }

    /* Ignore feval statements that are not worth the optimization */
    for (FevalInfo::SymToStatementsMap::iterator it = pFevalInfo->ConstantFirstArg.begin(),
            end = pFevalInfo->ConstantFirstArg.end(); it != end;) {
        bool inLoop = false;
        for (FevalInfo::FevalCallInfo* callInfo : it->second) {
            inLoop |= (callInfo->loopIdx > 0);
        }
        if (!inLoop) {
            pFevalInfo->ConstantFirstArg.erase(it++);
        } else {
            for (FevalInfo::FevalCallInfo* callInfo : it->second) {
                AssignStmt* pAssignStmt = (AssignStmt*) callInfo->assStmt;
                pFevalInfo->ParamExpressions.insert((ParamExpr*) pAssignStmt->getRightExpr());
            }
            ++it;
        }
    }

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        pFevalInfo->printResults();
    }

    return pFevalInfo;
}

void FevalInfo::printResults() {
    if (this->toOptimize()) {
        std::cerr << "=============================" << std::endl;
        std::cerr << "Feval statements to optimize:" << std::endl;

        for (FevalInfo::SymToStatementsMap::iterator it = ConstantFirstArg.begin(),
                end = ConstantFirstArg.end(); it != end; ++it) {
            std::cerr << std::endl << "Symbol for the call(s): " << it->first->toString() << std::endl;
            std::cerr << "Feval instruction(s) using the symbol as first argument:" << std::endl;
            for (FevalInfo::FevalCallInfo* callInfo : it->second) {
                std::cerr << "[@loopDepth" << callInfo->loopIdx << "] " << callInfo->assStmt->toString() << std::endl;
            }
        }

        std::cerr << "=============================" << std::endl;
    } else if (this->containsFevalInstructions) {
        std::cerr << "The function does not contain feval instructions that I can optimize." << std::endl;
    } else {
        std::cerr << "The function does not contain any feval instruction!" << std::endl;
    }
}