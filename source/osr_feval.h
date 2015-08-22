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
#include "objects.h"
#include "symbolexpr.h"
#include "../OSR/LLVMUtils.hpp"
#include "../OSR/OSRLibrary.hpp"

#include <map>
#include <set>
#include <utility>
#include <llvm/IR/Value.h>

class OSRFeval {
public:
    typedef std::map<SymbolExpr*, JITCompiler::Value*> IIRVarMap;

    typedef struct FevalInfoForOSR {
        ParamExpr*      pParamExpr;
        IIRVarMap*      varMap;
        llvm::Value*    arrayObjCreateInst;
        std::vector<std::pair<llvm::Value*, llvm::Value*>>   argsInArrayObj;
        llvm::Value*    interpreterCallInst;

        FevalInfoForOSR() : varMap(new IIRVarMap()) {}
        // TODO: check why if I create a new IIRVarMap in JITCompiler I get a random segfault later

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
            std::cerr << "VariableMap" << std::endl;
            for (IIRVarMap::iterator it = varMap->begin(), end = varMap->end(); it != end; ++it) {
                std::cerr << "[" << it->first->getSymName() << "]" << std::endl;
                std::cerr << "--> Type:  " << DataObject::getTypeName(it->second->objType) << std::endl;
                std::cerr << "--> Value: "; it->second->pValue->dump();
            }
        }
    } FevalInfoForOSR;

    typedef struct FevalInfoForOSRGen {
        FevalInfoForOSR*    fevalInfoForOSR;
        llvm::Value*        arg1;
        llvm::Value*        arg2;
        llvm::Value*        environment;
        std::vector<llvm::Value*>*  passedValues;
        JITCompiler::CompFunction*  pCompFunction;
        JITCompiler::CompVersion*   pCompVersion;
    } FevalInfoForOSRGen;

    typedef std::set<ParamExpr*> LocForOSRPoints;

    typedef std::pair<JITCompiler::CompFunction*, JITCompiler::CompVersion*> CompPair;
    typedef std::map<CompPair, std::vector<FevalInfoForOSR*>> CompPairToOSRInfoMap;
    typedef std::map<CompPair, LocForOSRPoints> CompPairToOSRPoints;
    typedef std::map<CompPair, LLVMUtils::ClonedFunc> CompPairToCtrlFun;

    static FevalInfoForOSR* createFevalInfoForOSR(JITCompiler::CompFunction* pCompFunction,
        JITCompiler::CompVersion* pCompVersion);

    static bool processCompVersion(JITCompiler::CompFunction* pCompFunction,
        JITCompiler::CompVersion* pCompVersion);

    static CompPairToOSRInfoMap CompOSRInfoMap;
    static CompPairToOSRPoints  CompOSRLocMap;
    static CompPairToCtrlFun    CompOSRCtrlFunMap;

private:
    typedef std::pair<ProgFunction*, std::vector<ParamExpr*>*> OptimizedFunPair;
    static LocForOSRPoints computeLocationsForOSRPoints(FevalInfo* analysisInfo);
    static LocForOSRPoints& getLocationsForOSRPoints(CompPair funPair);

    static void* funGenerator(OSRLibrary::RawOpenOSRInfo *info, void* profDataAddr);
    static OSRLibrary::OSRCond generateDefaultOSRCond();
    static OptimizedFunPair generateIIRFunc(ProgFunction* orig, Function* calledFunc,
        OSRFeval::FevalInfoForOSRGen* genInfo);
    static void parseClonedFunForIIRMapping(StmtSequence* origSeq, StmtSequence* clonedSeq,
        std::set<AssignStmt*> &origStmtsToMatch, std::map<AssignStmt*, AssignStmt*> &mapNewToOldSmts);
    static llvm::Function* generateIRforFunction(ProgFunction* pFunc, JITCompiler::CompFunction* pOldCompFunc,
        JITCompiler::CompVersion* pOldCompVersion);
};

#endif

