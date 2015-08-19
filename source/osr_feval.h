/* ===============================================================
 * Optimization of feval instructions in the code through OSR.
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */

#ifndef OSR_FEVAL_H
#define	OSR_FEVAL_H

#include "analysis_feval.h"
#include "paramexpr.h"
#include "jitcompiler.h"
#include <llvm/IR/Value.h>
#include <map>
#include <set>
#include <utility>

class OSRFeval {
public:
    typedef struct FevalInfoForOSR {
        ParamExpr*      pParamExpr;
        llvm::Value*    arrayObjCreateInst;
        std::vector<std::pair<llvm::Value*, llvm::Value*>>   argsInArrayObj;
        llvm::Value*    interpreterCallInst;

        void dump() {
            std::cerr << "ParamExpr: " << pParamExpr->toString() << std::endl;
            std::cerr << "ArrayObjCreateInst: "; arrayObjCreateInst->dump();
            size_t argIndex = 1;
            for (std::pair<llvm::Value*, llvm::Value*> &pair: argsInArrayObj) {
                std::cerr << "Argument [" << argIndex++ << "]:" << std::endl;
                std::cerr << "Expr: "; pair.first->dump();
                std::cerr << "addObject: "; pair.second->dump();
            }
            std::cerr << "InterpreterCallInst: "; interpreterCallInst->dump();
        }
    } FevalInfoForOSR;

    typedef std::set<ParamExpr*> LocForOSRPoints;

    typedef std::pair<JITCompiler::CompFunction*, JITCompiler::CompVersion*> CompPair;
    typedef std::map<CompPair, std::vector<FevalInfoForOSR*>> CompPairToOSRInfoMap;
    typedef std::map<CompPair, LocForOSRPoints> CompPairToOSRPoints;

    static FevalInfoForOSR* createFevalInfoForOSR(JITCompiler::CompFunction* pFunction,
        JITCompiler::CompVersion* pVersion);

    static LocForOSRPoints& getLocationsForOSRPoints(JITCompiler::CompFunction* pFunction,
        JITCompiler::CompVersion* pVersion);

    static CompPairToOSRInfoMap CompOSRInfoMap;
    static CompPairToOSRPoints  CompOSRLocMap;

private:
    static LocForOSRPoints computeLocationsForOSRPoints(FevalInfo* analysisInfo);
};

#endif

