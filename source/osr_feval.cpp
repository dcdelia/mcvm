/* ===============================================================
 * Optimization of feval instructions in the code through OSR.
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */
#include "osr_feval.h"

#include "analysis_feval.h"
#include "jitcompiler.h"
#include "paramexpr.h"
#include "../OSR/LLVMUtils.hpp"
#include "../OSR/Liveness.hpp"
#include "../OSR/OSRLibrary.hpp"
#include "mcvmstdlib.h"
#include "chararrayobj.h"
#include "interpreter.h"

#include <vector>
#include <utility>
#include <llvm/Analysis/Passes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>
#include <llvm/PassManager.h>
#include <llvm/Transforms/Scalar.h>

// static fields
OSRFeval::CompPairToOSRInfoMap  OSRFeval::CompOSRInfoMap;
OSRFeval::CompPairToOSRPoints   OSRFeval::CompOSRLocMap;
OSRFeval::CompPairToCtrlFun     OSRFeval::CompOSRCtrlFunMap;

OSRFeval::FevalInfoForOSR* OSRFeval::createFevalInfoForOSR(JITCompiler::CompFunction* pCompFunction,
        JITCompiler::CompVersion* pCompVersion) {
    FevalInfoForOSR* info = new FevalInfoForOSR();

    CompPair funPair(pCompFunction, pCompVersion);
    CompPairToOSRInfoMap::iterator cpIt = CompOSRInfoMap.find(funPair);
    if (cpIt == CompOSRInfoMap.end()) {
        CompOSRInfoMap.insert(std::pair<CompPair, std::vector<FevalInfoForOSR*>>
                                (funPair, std::vector<FevalInfoForOSR*>()));
        cpIt = CompOSRInfoMap.find(funPair);
    }
    cpIt->second.push_back(info);

    return info;
}

OSRFeval::LocForOSRPoints& OSRFeval::getLocationsForOSRPoints(OSRFeval::CompPair funPair) {
    if (CompOSRLocMap.count(funPair) == 0) {
        LocForOSRPoints locs = computeLocationsForOSRPoints(const_cast<FevalInfo*>(funPair.second->pFevalInfo));
        CompOSRLocMap.insert(std::pair<CompPair, LocForOSRPoints>(funPair, std::move(locs)));
    }

    return CompOSRLocMap[funPair];
}

OSRFeval::LocForOSRPoints OSRFeval::computeLocationsForOSRPoints(FevalInfo* analysisInfo) {
    OSRFeval::LocForOSRPoints locs;

    // parse one group at a time
    std::vector<FevalInfo::FevalCallInfo*> tmpVec;
    for (FevalInfo::SymToStatementsMap::iterator grpIt = analysisInfo->ConstantFirstArg.begin(),
            grpEnd = analysisInfo->ConstantFirstArg.end(); grpIt != grpEnd; ++grpIt) {
        std::vector<FevalInfo::FevalCallInfo*> &vec = grpIt->second;

        size_t vecSize = vec.size();
        locs.insert(vec[0]->pExpr);

        if (vecSize == 1) continue;

        // dominator-like analysis (results might be incomplete)
        tmpVec.push_back(vec[0]);
        for (size_t index = 1; index < vecSize; ++index) {
            FevalInfo::FevalCallInfo* cur = vec[index];
            bool dominated = false;
            for (FevalInfo::FevalCallInfo* prev: tmpVec) {
                StmtSequence *curSeq = cur->parentStmtSeq;
                StmtSequence *prevSeq = prev->parentStmtSeq;

                while (prevSeq != curSeq) {
                    curSeq = analysisInfo->ParentMap[curSeq];
                    if (curSeq == nullptr) break;
                }

                if (prevSeq == curSeq) {
                    dominated = true;
                    break;
                }
            }
            if (!dominated) {
                tmpVec.push_back(cur);
                locs.insert(cur->pExpr);
            }
        }

        tmpVec.clear();
    }

    return locs;
}

bool OSRFeval::processCompVersion(JITCompiler::CompFunction* pCompFunction, JITCompiler::CompVersion* pCompVersion) {
    CompPair funPair(pCompFunction, pCompVersion);
    //FevalInfo* fevalInfo = const_cast<FevalInfo*>(pCompVersion->pFevalInfo);
    LocForOSRPoints& locations = getLocationsForOSRPoints(funPair);

    CompPairToOSRInfoMap::iterator cpIt = CompOSRInfoMap.find(funPair);
    if (cpIt == CompOSRInfoMap.end()) return false;

    std::cerr << "Current function contains annotated feval instructions!" << std::endl;
    if (ConfigManager::s_veryVerboseVar) {
        for (FevalInfoForOSR *info: cpIt->second) {
            info->dump();
        }
    }

    std::cerr << "OSR points should be inserted at these instructions:" << std::endl;
    for (ParamExpr* loc: locations) {
        std::cerr << loc->toString() << std::endl;
    }

    llvm::Function* currFunction = pCompVersion->pLLVMFunc;
    llvm::Module* currModule = currFunction->getParent();
    assert(currModule != nullptr);

    // store the control version of the function
    LLVMUtils::ClonedFunc clonedFunPair = LLVMUtils::cloneFunction(currFunction);
    CompOSRCtrlFunMap.insert(std::pair<CompPair, LLVMUtils::ClonedFunc>(funPair, std::move(clonedFunPair)));

    // simplify CFG (too many BBs at this stage)
    llvm::FunctionPassManager FPM(currModule);
    FPM.add(llvm::createCFGSimplificationPass());
    FPM.doInitialization();
    FPM.run(*currFunction);

    /* Process each candidate location for OSR point insertion */
    std::vector<FevalInfoForOSR*> &fevalInfoForOSRVec = CompOSRInfoMap[funPair];

    // split basic blocks where needed
    std::vector<std::pair<FevalInfoForOSR*, llvm::BasicBlock*>> splitInfoVec;
    for (FevalInfoForOSR* info: fevalInfoForOSRVec) {
        // check if the feval is a candidate location
        if (locations.count(info->pParamExpr) > 0) {
            // split the basic block at the first feval-related instruction
            llvm::Instruction* arrayObjCreate = llvm::cast<llvm::Instruction>(info->arrayObjCreateInst);
            llvm::BasicBlock* currBB = arrayObjCreate->getParent();
            llvm::BasicBlock* splitBB = currBB->splitBasicBlock(arrayObjCreate, "splitForOSR");
            splitInfoVec.push_back(std::pair<FevalInfoForOSR*, llvm::BasicBlock*>(info, splitBB));
        }
    }

    LivenessAnalysis livenessInfo(currFunction);

    for (std::pair<FevalInfoForOSR*, llvm::BasicBlock*> &splitPair: splitInfoVec) {
        FevalInfoForOSR* info = splitPair.first;
        llvm::BasicBlock* splitBB = splitPair.second;

        std::vector<llvm::Value*>* valuesToTransfer = OSRLibrary::defaultValuesToTransferForOpenOSR(livenessInfo, *splitBB);

        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "Value* to transfer for instruction " << info->pParamExpr->toString() << std::endl;
            for (llvm::Value* v: *valuesToTransfer) {
                std::cerr << (void*)v; v->dump();
            }

            std::cerr << "Available Value* from VarMap:" << std::endl;
            for (IIRVarMap::iterator it = info->varMap->begin(), end = info->varMap->end(); it != end; ++it) {
                JITCompiler::Value *jitVal = it->second;
                std::cerr << "[" << it->first->getSymName() << "] " << (void*)jitVal->pValue << std::endl;
            }

            /*
            std::cerr << "Info from FevalInfoForOSR:" << std::endl;
            std::cerr << "arguments:" << std::endl;
            for (auto &pair: info->argsInArrayObj) {
                std::cerr << (void*)pair.first << " && " << (void*)pair.second << std::endl;
            }
            std::cerr << "" << std::endl;
            std::cerr << "call to interpreter: " << (void*)info->interpreterCallInst << std::endl;
            */

            std::cerr << "environment: " << (void*)pCompVersion->pEnvObject << std::endl;
            // Note that &mcvm::stdlib::feval is the first arg for Interpreter::callFunction()

            std::cerr << "Function arguments:" << std::endl;
            for (llvm::Function::arg_iterator argIt = currFunction->arg_begin(),
                    argEnd = currFunction->arg_end(); argIt != argEnd; ++argIt) {
                llvm::Value* arg = argIt;
                std::cerr << (void*)arg; arg->dump();
            }
        }

        // prepare FevalInfoForOSRGen object
        FevalInfoForOSRGen* infoForOSRGen = new FevalInfoForOSRGen();
        infoForOSRGen->fevalInfoForOSR = info;
        infoForOSRGen->pCompFunction = pCompFunction;
        infoForOSRGen->pCompVersion = pCompVersion;
        infoForOSRGen->environment = pCompVersion->pEnvObject;
        assert(currFunction->getArgumentList().size() == 2);
        llvm::Function::arg_iterator argIt = currFunction->arg_begin();
        infoForOSRGen->arg1 = argIt++;
        infoForOSRGen->arg2 = argIt++;
        infoForOSRGen->passedValues = valuesToTransfer;

        // prepare OpenOSRInfo object for insertOpenOSR
        OSRLibrary::OpenOSRInfo openOSRInfo;
        openOSRInfo.f1 = currFunction;
        openOSRInfo.b1 = splitBB;
        openOSRInfo.extra = infoForOSRGen;

        OSRLibrary::DestFunGenerator generator = OSRFeval::funGenerator;
        OSRLibrary::OSRCond cond = generateDefaultOSRCond();

        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "Value* to transfer for instruction " << info->pParamExpr->toString() << std::endl;
            size_t valIndex = 0;
            for (llvm::Value* v: *valuesToTransfer) {
                std::cerr << "[" << valIndex++ << "] " << (void*)v << " "; v->dump();
            }
        }

        // determine which llvm::Value corresponds to the first argument for the feval call
        std::pair<llvm::Value*, llvm::Value*> &firstPairForArrayObj = info->argsInArrayObj[0];
        llvm::Value* profVal = firstPairForArrayObj.first;
        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "First argument for feval: [" << (void*)profVal << "] "; profVal->dump();
        }

        OSRLibrary::OSRPair retOSRPair = OSRLibrary::insertOpenOSR(*JITCompiler::s_Context, openOSRInfo, cond,
                                            profVal, generator, true, currFunction->getName(), valuesToTransfer);

        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
            //retOSRPair.first->dump(); // updated currFunction
            retOSRPair.second->dump(); // generated stub
        }
    }

    return true;
}

OSRLibrary::OSRCond OSRFeval::generateDefaultOSRCond() {
    OSRLibrary::OSRCond cond;
    llvm::ConstantFP* one = llvm::ConstantFP::get(*JITCompiler::s_Context, llvm::APFloat(1.0));
    cond.push_back(new llvm::FCmpInst(llvm::CmpInst::FCMP_TRUE, one, one, "alwaysOSR"));

    return cond;
}

void* OSRFeval::funGenerator(OSRLibrary::RawOpenOSRInfo *info, void* profDataAddr) {
    std::cerr << "Hi! I will generate optimized code on-the-fly :-)" << std::endl;

    llvm::Function* srcFun = (llvm::Function*)info->f1;
    llvm::BasicBlock* srcB = (llvm::BasicBlock*)info->b1;

    FevalInfoForOSRGen* genInfo = (FevalInfoForOSRGen*)info->extra;
    FevalInfoForOSR* infoAtIIR = genInfo->fevalInfoForOSR;

    std::cerr << "Analyzing passed values..." << std::endl;
    bool error = false;
    for (size_t index = 0, size = genInfo->passedValues->size(); index < size; ++index) {
        llvm::Value* v = (*(genInfo->passedValues))[index];
        std::cerr << "[" << index << "] with address " << (void*)v << std::endl;
        std::cerr << "--> ";
        // try to match against environment or arguments
        if (v == genInfo->environment) {
            std::cerr << "environment object" << std::endl;
        } else if (v == genInfo->arg1) {
            std::cerr << "argument 1 (input)" << std::endl;
        } else if (v == genInfo->arg2) {
            std::cerr << "argument 2 (output)" << std::endl;
        } else {
            // try to match against IIR variables
            for (IIRVarMap::iterator it = infoAtIIR->varMap->begin(),
                    end = infoAtIIR->varMap->end(); it != end; ) {
                SymbolExpr* sym = it->first;
                JITCompiler::Value* jitVal = it->second;
                if (it->second->pValue == v) {
                    std::cerr << "variable " << sym->getSymName() << " of type "
                            << DataObject::getTypeName(jitVal->objType) << std::endl;
                    break;
                }
                if (++it == end) {
                    std::cerr << "UNABLE TO MATCH VALUE!!!" << std::endl;
                    error = true;
                }
            }
        }
    }
    assert(!error);

    DataObject* argForFeval = (DataObject*)profDataAddr;
    DataObject::Type argForFevalType = argForFeval->getType();

    std::cerr << "Analyzing passed profiling value..." << std::endl;
    std::cerr << "--> argument has type " << DataObject::getTypeName(argForFevalType) << std::endl;
    std::cerr << "--> string representation: " << argForFeval->toString() << std::endl;

    Function* calledIIRFunc;

    // see fevalFunc() in mcvmstdblib.cpp
    if (argForFevalType == DataObject::FN_HANDLE) {
        FnHandleObj* pHandle = (FnHandleObj*)argForFeval;
        calledIIRFunc = pHandle->getFunction();
    } else if (argForFevalType == DataObject::CHARARRAY) {
        CharArrayObj* pCharArray = (CharArrayObj*)argForFeval;

        // see callByName() in interpreter.cpp
        DataObject* pObject = Interpreter::evalSymbol(SymbolExpr::getSymbol(pCharArray->getString()), &Interpreter::s_globalEnv);
        if (pObject->getType() != DataObject::FUNCTION) {
            throw RunError("symbol is not bound to a function");
        }
        calledIIRFunc = (Function*)pObject;
    } else {
        throw RunError("can only apply feval to function handles or names");
    }

    std::cerr << "Function to call: " << calledIIRFunc->getFuncName() << std::endl;

    // generate IIR function where feval calls are replaced with direct calls
    OptimizedFunPair optPair = generateIIRFunc((ProgFunction*)genInfo->pCompFunction->pProgFunc, calledIIRFunc, genInfo);

    assert(false);
    return nullptr;
}

OSRFeval::OptimizedFunPair OSRFeval::generateIIRFunc(ProgFunction* orig, Function* calledFunc,
        OSRFeval::FevalInfoForOSRGen* genInfo) {
    FevalInfo* analysis = const_cast<FevalInfo*>(genInfo->pCompVersion->pFevalInfo);
    ParamExpr* pExpr = genInfo->fevalInfoForOSR->pParamExpr;

    // determine which symbols we should manipulate
    SymbolExpr* pSymForFeval = const_cast<SymbolExpr*>((const SymbolExpr*)pExpr->getArgument(0));
    SymbolExpr* pSymToCall = SymbolExpr::getSymbol(calledFunc->getFuncName());

    if (ConfigManager::s_veryVerboseVar) {
        std::cerr << "Symbol to replace in feval: " << pSymForFeval->getSymName() << std::endl;
        std::cerr << "Symbol to call directly: " << pSymToCall->getSymName() << std::endl;
    }

    // determine which statements should be replaced
    std::vector<FevalInfo::FevalCallInfo*> &vecFevalCallInfo = analysis->ConstantFirstArg[pSymForFeval];
    std::set<AssignStmt*> assignStmts;
    for (FevalInfo::FevalCallInfo* info: vecFevalCallInfo) {
        assignStmts.insert(info->assStmt);
    }

    /* Clone IIR function and update it */
    ProgFunction* newFun = orig->copyWithCurrentBody();
    newFun->setFuncName("OSR_" + orig->getFuncName());

    std::map<AssignStmt*, AssignStmt*> mapNewToOldAssignSmts;
    std::vector<ParamExpr*> *newParamExprs = new std::vector<ParamExpr*>();

    parseClonedFunForIIRMapping(orig->getCurrentBody(), newFun->getCurrentBody(), assignStmts, mapNewToOldAssignSmts);
    assert(assignStmts.empty());

    for (std::map<AssignStmt*, AssignStmt*>::iterator it = mapNewToOldAssignSmts.begin(),
            end = mapNewToOldAssignSmts.end(); it != end; ++it) {
        AssignStmt* pNewStmt = it->first;
        //AssignStmt* pOldStmt = it->second;

        // we checked that the type is Expression::PARAM during the analysis phase
        ParamExpr* pExpr = (ParamExpr*)pNewStmt->getRightExpr();

        const Expression::ExprVector& oldArgs = pExpr->getArguments();
        Expression::ExprVector newArgs; // TODO assert check on pSymForFeval vs oldArgs[0]?
        for (size_t index = 1, numArgs = oldArgs.size(); index < numArgs; ++index) {
            newArgs.push_back(oldArgs[index]);
        }

        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "Old expr: " << pExpr->toString() << std::endl;
        }

        // replace feval with direct call and set new args
        pExpr->replaceSubExpr(0, pSymToCall);
        pExpr->replaceArgs(std::move(newArgs));

        newParamExprs->push_back(pExpr); // TODO perhaps we need assignments?

        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "New expr: " << pExpr->toString() << std::endl;
        }
    }

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << orig->toString();
    }

    std::cerr << newFun->toString();

    return OptimizedFunPair(newFun, newParamExprs);
}

void OSRFeval::parseClonedFunForIIRMapping(StmtSequence* origSeq, StmtSequence* clonedSeq,
        std::set<AssignStmt*> &origStmtsToMatch, std::map<AssignStmt*, AssignStmt*> &mapNewToOldSmts) {
    StmtSequence::StmtVector& origStmts = const_cast<StmtSequence::StmtVector&>(origSeq->getStatements());
    StmtSequence::StmtVector& newStmts = const_cast<StmtSequence::StmtVector&>(clonedSeq->getStatements());

    size_t numStmts = origStmts.size();
    //assert(numStmts == newStmts.size());
    for (size_t index = 0; index < numStmts; ++index) {
        Statement* pOldStmt = origStmts[index];

        switch (pOldStmt->getStmtType()) {
            case Statement::ASSIGN:
            {
                AssignStmt* pOldAssignStmt = (AssignStmt*) pOldStmt;
                std::set<AssignStmt*>::iterator it = origStmtsToMatch.find(pOldAssignStmt);
                if (it != origStmtsToMatch.end()) {
                    AssignStmt* pNewAssignStmt = (AssignStmt*) newStmts[index];
                    mapNewToOldSmts.insert(std::pair<AssignStmt*, AssignStmt*>(pNewAssignStmt, pOldAssignStmt));
                    origStmtsToMatch.erase(it);
                    if (origStmtsToMatch.empty()) return;
                }
            }
            break;

            case Statement::IF_ELSE:
            {
                IfElseStmt* pOldIfElseStmt = (IfElseStmt*) pOldStmt;
                IfElseStmt* pNewIfElseStmt = (IfElseStmt*) newStmts[index];

                parseClonedFunForIIRMapping(pOldIfElseStmt->getIfBlock(), pNewIfElseStmt->getIfBlock(),
                                            origStmtsToMatch, mapNewToOldSmts);
                if (origStmtsToMatch.empty()) return;
                parseClonedFunForIIRMapping(pOldIfElseStmt->getElseBlock(), pNewIfElseStmt->getElseBlock(),
                                            origStmtsToMatch, mapNewToOldSmts);
                if (origStmtsToMatch.empty()) return;
            }
            break;

            case Statement::LOOP:
            {
                LoopStmt* pOldLoopStmt = (LoopStmt*) pOldStmt;
                LoopStmt* pNewLoopStmt = (LoopStmt*) newStmts[index];
                parseClonedFunForIIRMapping(pOldLoopStmt->getBodySeq(), pNewLoopStmt->getBodySeq(),
                                            origStmtsToMatch, mapNewToOldSmts);
                if (origStmtsToMatch.empty()) return;
            }
            break;

            default:
            {
                // nothing to do
            }
        }
    }
}