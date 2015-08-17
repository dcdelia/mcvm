/* ===============================================================
 * Analysis of feval instructions in the code.
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */

#ifndef ANALYSIS_FEVAL_H
#define	ANALYSIS_FEVAL_H

#include "analysismanager.h"
#include "analysis_reachdefs.h"
#include "functions.h"
#include "stmtsequence.h"
#include "typeinfer.h"

#include <vector>
#include <map>

class FevalInfo : public AnalysisInfo {
    public:
        typedef struct FevalCallInfo {
            Statement* stmt;
            int loopIdx;
            IIRNode* parent;
            Expression* firstArg;
            VarDefMap* reachDefs;
        } FevalCallInfo;

        typedef std::vector<FevalCallInfo*> FevalCallsVec;
        typedef std::map<SymbolExpr*, std::vector<FevalCallInfo*>> SymToStatementsMap;

        FevalCallsVec FevalCalls;
        SymToStatementsMap ConstantFirstArg;
        bool containsFevalInstructions = false;

        bool shouldOptimize() { return !ConstantFirstArg.empty(); }
        void printResults();
};

AnalysisInfo* computeFevalInfo(
	const ProgFunction* pFunction,
	const StmtSequence* pFuncBody,
	const TypeSetString& inArgTypes,
	bool returnBottom
);

#endif

