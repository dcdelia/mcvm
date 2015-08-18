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
#include "paramexpr.h"

#include <map>
#include <set>
#include <utility>
#include <vector>

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
        std::set<ParamExpr*> ParamExpressions;

        bool toOptimize() { return !ConstantFirstArg.empty(); }
        bool toTrack(ParamExpr* pExpr) {
            return ParamExpressions.find(pExpr) != ParamExpressions.end();
        }
        void printResults();

        typedef std::pair<const ProgFunction*, const StmtSequence*> FunMapKey;

        static std::map<FunMapKey, FevalInfo*> FevalInfoMap;


};

AnalysisInfo* computeFevalInfo(
	const ProgFunction* pFunction,
	const StmtSequence* pFuncBody,
	const TypeSetString& inArgTypes,
	bool returnBottom
);

#endif

