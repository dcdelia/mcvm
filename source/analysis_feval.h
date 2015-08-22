/* ===============================================================
 * Analysis of feval instructions in the code.
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */

#ifndef ANALYSIS_FEVAL_H
#define	ANALYSIS_FEVAL_H

#include "analysismanager.h"
#include "analysis_reachdefs.h"
#include "assignstmt.h"
#include "functions.h"
#include "stmtsequence.h"
#include "typeinfer.h"
#include "paramexpr.h"

#include <map>
#include <set>
#include <utility>
#include <vector>

class FevalAnalysisInfo : public AnalysisInfo {
    public:
        typedef struct FevalCallInfo {
            AssignStmt* assStmt;
            ParamExpr*  pExpr;
            int loopIdx;
            StmtSequence* parentStmtSeq;
            const Expression* firstArg;
            VarDefMap* reachDefs;
        } FevalCallInfo;

        typedef struct OptimizedCallInfo {
            AssignStmt* assStmt;
            ParamExpr*  pExpr;
        } OptimizedCallInfo;

        typedef std::vector<FevalCallInfo*> FevalCallInfoVec;
        typedef std::map<SymbolExpr*, std::vector<FevalCallInfo*>> SymToFevalCallInfosMap;
        typedef std::map<StmtSequence*, StmtSequence*> StmtSeqToStmtSeqMap;

        // set of FevalCallInfo objects associated with *all* feval instructions
        FevalCallInfoVec FevalCalls;

        // FevalCallInfo objects associated with a SymbolExpr that is a read-only argument
        SymToFevalCallInfosMap ConstantFirstArg;

        // set of ParamExpr objects containing a feval instruction to optimize
        std::set<ParamExpr*> FevalParamExprs;

        // set of ParamExpr objects for which a feval instruction has been optimized away
        std::set<ParamExpr*> OptimizedParamExprs;

        // used for quick inspection by JITCompiler
        bool containsParamExprsToTrack = false;

        // used for cloning, stores tree edges across StmtSequence objects
        StmtSeqToStmtSeqMap ParentMap;

        // results for this analysis do not depend on inArgTypes!
        typedef std::pair<const ProgFunction*, const StmtSequence*> KeyForFun;
        static std::map<KeyForFun, FevalAnalysisInfo*> FevalInfoMap;

        /* Querying methods */
        bool toOptimize() {
            return !ConstantFirstArg.empty();
        }

        bool isFevalToTrack(ParamExpr* pExpr) {
            return FevalParamExprs.find(pExpr) != FevalParamExprs.end();
        }

        bool isOptimizedCallToTrack(ParamExpr* pExpr) {
            return OptimizedParamExprs.find(pExpr) != OptimizedParamExprs.end();
        }

        void printResults();
};

AnalysisInfo* computeFevalInfo(
	const ProgFunction* pFunction,
	const StmtSequence* pFuncBody,
	const TypeSetString& inArgTypes,
	bool returnBottom
);

#endif

