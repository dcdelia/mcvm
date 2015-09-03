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
#include <llvm/IR/Verifier.h>
#include <llvm/IR/CFG.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/CFG.h>
#include <llvm/ExecutionEngine/GenericValue.h>

// static fields
OSRFeval::CompPairToOSRFevalInfoMap OSRFeval::CompOSRInfoMap;
OSRFeval::CompPairToOSRPoints       OSRFeval::CompOSRLocMap;
OSRFeval::CompPairToCtrlFun         OSRFeval::CompOSRCtrlFunMap;
OSRFeval::CompPairToOSROptInfoMap   OSRFeval::CompOSROptInfoMap;
OSRFeval::CodeCacheMap              OSRFeval::CodeCache;

OSRFeval::FevalInfoForOSR* OSRFeval::createFevalInfoForOSR(JITCompiler::CompFunction* pCompFunction,
        JITCompiler::CompVersion* pCompVersion) {
    FevalInfoForOSR* info = new FevalInfoForOSR();

    CompPair funPair(pCompFunction, pCompVersion);
    CompPairToOSRFevalInfoMap::iterator cpIt = CompOSRInfoMap.find(funPair);
    if (cpIt == CompOSRInfoMap.end()) {
        CompOSRInfoMap.insert(std::pair<CompPair, std::vector<FevalInfoForOSR*>>
                                (funPair, std::vector<FevalInfoForOSR*>()));
        cpIt = CompOSRInfoMap.find(funPair);
    }
    cpIt->second.push_back(info);

    return info;
}

OSRFeval::OptimizedFevalInfoForOSR* OSRFeval::createOptimizedFevalInfoForOSR(
        JITCompiler::CompFunction* pCompFunction, JITCompiler::CompVersion* pCompVersion) {
    OptimizedFevalInfoForOSR* info = new OptimizedFevalInfoForOSR();
    //info->varMap = new IIRVarMap();

    CompPair funPair(pCompFunction, pCompVersion);
    CompPairToOSROptInfoMap::iterator cpIt = CompOSROptInfoMap.find(funPair);

    if (cpIt == CompOSROptInfoMap.end()) {
        CompOSROptInfoMap.insert(std::pair<CompPair, std::vector<OptimizedFevalInfoForOSR*>>
                                (funPair, std::vector<OptimizedFevalInfoForOSR*>()));
        cpIt = CompOSROptInfoMap.find(funPair);
    }
    cpIt->second.push_back(info);

    return info;
}

OSRFeval::LocForOSRPoints& OSRFeval::getLocationsForOSRPoints(OSRFeval::CompPair funPair) {
    if (CompOSRLocMap.count(funPair) == 0) {
        LocForOSRPoints locs = computeLocationsForOSRPoints(const_cast<FevalAnalysisInfo*>(funPair.second->pFevalInfo));
        CompOSRLocMap.insert(std::pair<CompPair, LocForOSRPoints>(funPair, std::move(locs)));
    }

    return CompOSRLocMap[funPair];
}

OSRFeval::LocForOSRPoints OSRFeval::computeLocationsForOSRPoints(FevalAnalysisInfo* analysisInfo) {
    OSRFeval::LocForOSRPoints locs;

    // parse one group at a time
    std::vector<FevalAnalysisInfo::FevalCallInfo*> tmpVec;
    for (FevalAnalysisInfo::SymToFevalCallInfosMap::iterator grpIt = analysisInfo->ConstantFirstArg.begin(),
            grpEnd = analysisInfo->ConstantFirstArg.end(); grpIt != grpEnd; ++grpIt) {
        std::vector<FevalAnalysisInfo::FevalCallInfo*> &vec = grpIt->second;

        size_t vecSize = vec.size();
        locs.insert(vec[0]->pExpr);

        if (vecSize == 1) continue;

        // dominator-like analysis (results might be incomplete)
        tmpVec.push_back(vec[0]);
        for (size_t index = 1; index < vecSize; ++index) {
            FevalAnalysisInfo::FevalCallInfo* cur = vec[index];
            bool dominated = false;
            for (FevalAnalysisInfo::FevalCallInfo* prev: tmpVec) {
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

    CompPairToOSRFevalInfoMap::iterator cpIt = CompOSRInfoMap.find(funPair);
    if (cpIt == CompOSRInfoMap.end()) return false;

    std::cerr << "Function contains annotated feval instructions!" << std::endl;
    if (ConfigManager::s_veryVerboseVar) {
        for (FevalInfoForOSR *info: cpIt->second) {
            info->dump();
        }
    }

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "OSR points should be inserted at these instructions:" << std::endl;
        for (ParamExpr* loc: locations) {
            std::cerr << loc->toString() << std::endl;
        }
    }

    llvm::Function* currFunction = pCompVersion->pLLVMFunc;
    llvm::Module* currModule = currFunction->getParent();
    assert(currModule != nullptr);

    /* // store the control version of the function
    LLVMUtils::ClonedFunc clonedFunPair = LLVMUtils::cloneFunction(currFunction);
    CompOSRCtrlFunMap.insert(std::pair<CompPair, LLVMUtils::ClonedFunc>(funPair, std::move(clonedFunPair))); */

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
                std::cerr << "[" << it->first->getSymName() << "] " << (void*)jitVal->pValue
                    << " of type: " << DataObject::getTypeName(jitVal->objType) << std::endl;
            }

            std::cerr << "Info from FevalInfoForOSR:" << std::endl;
            std::cerr << "arguments:" << std::endl;
            for (auto &pair: info->argsInArrayObj) {
                std::cerr << (void*)pair.first << " && " << (void*)pair.second << std::endl;
            }
            std::cerr << "" << std::endl;
            std::cerr << "call to interpreter: " << (void*)info->interpreterCallInst << std::endl;

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

        /* (verbose, updateF1, branchTakenProb, nameForNewF1, modForNewF1,
         * ptrForF1NewToF1Map, nameForNewF2, nameForNewF2, ptrForF2NewToF2Map) */
        OSRLibrary::OSRPointConfig config(false, true, -1, nullptr, currModule,
                nullptr, nullptr, nullptr, nullptr);

        OSRLibrary::OSRPair retOSRPair = OSRLibrary::insertOpenOSR(*JITCompiler::s_Context, openOSRInfo, cond,
                                            profVal, generator, valuesToTransfer, config);

        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
            retOSRPair.first->dump(); // updated currFunction
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

void OSRFeval::printInfoOnAvailableValues(OSRFeval::IIRVarMap &varMap, llvm::Function* fun, llvm::Value* env) {
    std::cerr << "Tracked variables for function " << fun->getName().str() << ":" << std::endl;

    llvm::Function::arg_iterator argIt = fun->arg_begin();
    llvm::Value* arg1 = argIt++;
    llvm::Value* arg2 = argIt++;

    std::cerr << "[env] "; env->dump();
    std::cerr << "[arg1] "; arg1->dump();
    std::cerr << "[arg2] "; arg2->dump();

    for (auto &pair: varMap) {
        JITCompiler::Value* jitVal = pair.second;
        std::cerr << "<" << pair.first->getSymName() << "> of type " << DataObject::getTypeName(jitVal->objType) << std::endl;
        std::cerr << "--> "; jitVal->pValue->dump();
    }

}

void OSRFeval::computePredecessorsForBlock(llvm::BasicBlock* B, std::set<llvm::BasicBlock*> &predecessors) {
    for (llvm::pred_iterator it = llvm::pred_begin(B), end = llvm::pred_end(B); it != end; ++it) {
        llvm::BasicBlock* currBlock = *it;
        if (predecessors.count(currBlock) == 0) {
            predecessors.insert(currBlock);
            computePredecessorsForBlock(currBlock, predecessors);
        }
    }

}

bool OSRFeval::hasNoPrevUses(llvm::Value* v, std::set<llvm::BasicBlock*> &predecessors) {
    for (llvm::User *U: v->users()) {
        if (llvm::Instruction* I = llvm::dyn_cast<llvm::Instruction>(U)) {
            llvm::BasicBlock* B = I->getParent();
            if (predecessors.count(B) != 0) {
                if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
                    std::cerr << "ERROR: Value " << v->getName().str() << " is used in previous instruction: "; I->dump();
                }
                return false;
            } else if (ConfigManager::s_veryVerboseVar) {
                std::cerr << "Value " << v->getName().str() << " is used in later istruction: "; I->dump();
            }
        }
    }

    return true;
}

bool OSRFeval::sanityCheckOnPassedValues(OSRFeval::FevalInfoForOSRGen* genInfo, llvm::BasicBlock* srcBlock,
        llvm::Function* srcFun) {
    bool error = false;

    std::set<llvm::BasicBlock*> predecessors;
    computePredecessorsForBlock(srcBlock, predecessors);
    bool inLoop = (predecessors.count(srcBlock) != 0);

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "Analyzing values passed at the OSR point..." << std::endl;
    }
    FevalInfoForOSR* infoAtIIR = genInfo->fevalInfoForOSR;
    for (size_t index = 0, size = genInfo->passedValues->size(); index < size; ++index) {
        llvm::Value* v = (*(genInfo->passedValues))[index];
        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "[" << index << "] with address " << (void*)v << " - "; v->dump();
            std::cerr << "--> ";
        }
        // try to match against environment or arguments
        if (v == genInfo->environment) {
            if (ConfigManager::s_veryVerboseVar) std::cerr << "environment object" << std::endl;
        } else if (v == genInfo->arg1) {
            if (ConfigManager::s_veryVerboseVar) std::cerr << "argument 1 (input)" << std::endl;
        } else if (v == genInfo->arg2) {
            if (ConfigManager::s_veryVerboseVar) std::cerr << "argument 2 (output)" << std::endl;
        } else {
            // try to match against IIR variables
            for (IIRVarMap::iterator it = infoAtIIR->varMap->begin(),
                    end = infoAtIIR->varMap->end(); it != end; ) {
                SymbolExpr* sym = it->first;
                JITCompiler::Value* jitVal = it->second;
                if (it->second->pValue == v) {
                    if (ConfigManager::s_veryVerboseVar) {
                    std::cerr << "variable " << sym->getSymName() << " of type "
                            << DataObject::getTypeName(jitVal->objType) << std::endl;
                    }
                    break;
                }
                if (++it == end) {
                    if (llvm::isa<llvm::AllocaInst>(v)) {
                        if (hasNoPrevUses(v, predecessors)) {
                            if (ConfigManager::s_veryVerboseVar) std::cerr << "alloca with no previous uses!" << std::endl;
                        } else if (inLoop) {
                            predecessors.erase(srcBlock);
                            if (hasNoPrevUses(v, predecessors)) {
                                // TODO: liveness analysis info ensures that I will be writing the values using it before reading them?!?
                                std::cerr << "WARNING: alloca referenced in current block (loop)" << std::endl;
                            } else {
                                std::cerr << "ERROR: alloca instruction already referenced in a predecessor block" << std::endl;
                                error = true;
                            }
                            predecessors.insert(srcBlock);
                        } else {
                            std::cerr << "ERROR: alloca instruction already referenced in a predecessor block" << std::endl;
                            error = true;
                        }
                    } else {
                        std::cerr << "ERROR: UNABLE TO MATCH VALUE!!!" << std::endl;
                        v->dump();
                        error = true;
                    }
                }
            }
        }
    }

    return !error;
}

void* OSRFeval::funGenerator(OSRLibrary::RawOpenOSRInfo *info, void* profDataAddr) {
    //std::cerr << "Hi! I will generate optimized code on-the-fly :-)" << std::endl;

    llvm::Function* srcFun = (llvm::Function*)info->f1;
    llvm::BasicBlock* srcB = (llvm::BasicBlock*)info->b1;

    FevalInfoForOSRGen* genInfo = (FevalInfoForOSRGen*)info->extra;
    FevalInfoForOSR* infoAtIIR = genInfo->fevalInfoForOSR;

    DataObject* argForFeval = (DataObject*)profDataAddr;
    DataObject::Type argForFevalType = argForFeval->getType();

    if (ConfigManager::s_veryVerboseVar) {
        std::cerr << "Analyzing passed profiling value..." << std::endl;
        std::cerr << "--> argument has type " << DataObject::getTypeName(argForFevalType) << std::endl;
        std::cerr << "--> string representation: " << argForFeval->toString() << std::endl;
    }

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

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "Function to call: " << calledIIRFunc->getFuncName() << std::endl;
    }

    // check code cache
    CodeCacheKey cacheKey(genInfo, calledIIRFunc);
    CodeCacheMap::iterator cacheIt = CodeCache.find(cacheKey);
    if (cacheIt != CodeCache.end()) {
        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
            std::cerr << "A previously jitted code is available!" << std::endl;
        }
        return cacheIt->second;
    }

    // check if we have all the required information about values
    bool safe = sanityCheckOnPassedValues(genInfo, srcB, srcFun);
    if (!safe) srcFun->dump();
    assert(safe);

    // generate IIR function where feval calls are replaced with direct calls
    OptimizedFunPair optPair = generateIIRFunc((ProgFunction*)genInfo->pCompFunction->pProgFunc, calledIIRFunc, genInfo);

    // lower IIR to LLVM IR
    std::pair<llvm::Function*, CompPair> IRPair = generateIRforFunction(optPair.first,
            genInfo->pCompFunction, genInfo->pCompVersion, optPair.second);
    llvm::Function* newFun = IRPair.first;
    CompPair &newCompPair = IRPair.second;
    llvm::Module* modForNewFun = newFun->getParent();

    // perform code verification
    if (ConfigManager::s_veryVerboseVar) {
        std::cerr << "Verifying optimized function..." << std::endl;
    }
    JITCompiler::verifyLLVMFunction(newFun);

    // generate continuation function
    llvm::Function* OSRDestFun = nullptr;

    ParamExpr* pOSRTriggerExpr = infoAtIIR->pParamExpr;
    ParamExpr* pOSRContinuationExpr = optPair.second[pOSRTriggerExpr];
    assert (pOSRContinuationExpr != nullptr);

    /* Generate state mapping and continuation function */
    std::pair<StateMap*, llvm::Function*> mapPair = generateContinuationFunction(srcFun,
        srcB, newFun, modForNewFun, newCompPair, genInfo, pOSRContinuationExpr);

    //StateMap* M = mapPair.first;
    OSRDestFun = mapPair.second;

    if (ConfigManager::s_veryVerboseVar) {
        std::cerr << "Generated continuation function (pre-optimization):" << std::endl;
        OSRDestFun->dump();
    }

    // now we can perform verification of OSRDestFun
    if (llvm::verifyFunction(*OSRDestFun, &llvm::outs())) {
        OSRDestFun->dump();
        OSRDestFun->viewCFGOnly();
    }

    // perform McVM default optimizations
    JITCompiler::runFPM(OSRDestFun);

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "Generated continuation function (post-optimization):" << std::endl;
        OSRDestFun->dump();
    }

    const std::string OSRDestFunName = OSRDestFun->getName().str();

    void* pFuncPtr = (void*)JITCompiler::s_pExecEngine->getFunctionAddress(OSRDestFunName);

    // TODO store pFuncPtr inside CompVersion?

    CodeCache[cacheKey] = pFuncPtr;

    return pFuncPtr;
}

std::pair<StateMap*, llvm::Function*> OSRFeval::generateContinuationFunction(llvm::Function* origFunc,
        llvm::BasicBlock* origBlock, llvm::Function* newFunc, llvm::Module* modForNewFun,
        OSRFeval::CompPair &newCompPair, OSRFeval::FevalInfoForOSRGen* OSRGenInfo, ParamExpr* pExpr) {

    FevalInfoForOSR* infoAtOldIIR = OSRGenInfo->fevalInfoForOSR;
    OptimizedFevalInfoForOSR *optInfo = nullptr;

    std::vector<OptimizedFevalInfoForOSR*> &pOptInfoVec = CompOSROptInfoMap[newCompPair];
    for (OptimizedFevalInfoForOSR *pCur: pOptInfoVec) {
        if (pCur->pExpr == pExpr) {
            optInfo = pCur;
            break;
        }
    }
    assert(optInfo != nullptr);

    // let's assume that using the empty entryBlock is fine
    llvm::BasicBlock* newBlock = optInfo->block;

    LivenessAnalysis liveForNewFun(newFunc);
    LivenessAnalysis::LiveValues& liveInAtNewBlock = liveForNewFun.getLiveInValues(newBlock);
    std::vector<llvm::Value*> *valuesToSet = StateMap::getValuesToSetForBlock(*newBlock, liveInAtNewBlock);

    // fetch environment and arguments
    llvm::Value* environment = newCompPair.second->pEnvObject;
    assert(newFunc->getArgumentList().size() == 2);
    llvm::Function::arg_iterator argIt = newFunc->arg_begin();
    llvm::Value* arg1 = argIt++;
    llvm::Value* arg2 = argIt++;

    // just for debugging purposes
    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "Destination block: "; newBlock->dump();

        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "Values to set:" << std::endl;
            for (llvm::Value* v: *valuesToSet) {
                v->dump();
            }
            printInfoOnAvailableValues(*optInfo->varMap, newFunc, environment);
        }
    }

    // create the StateMap object
    StateMap *M = new StateMap(origFunc, newFunc);
    StateMap::BlockPair blockPair(origBlock, newBlock);
    llvm::Value* oldEnv = OSRGenInfo->environment;

    std::map<SymbolExpr*, JITValPair> SymToIIRValPairMap;

    // info to reconstruct alloca instructions
    std::set<llvm::BasicBlock*> predecessors;
    computePredecessorsForBlock(newBlock, predecessors);
    bool inLoop = (predecessors.count(newBlock) != 0);
    std::vector<llvm::AllocaInst*> allocasToAdd;

    for (llvm::Value* valueToSet: *valuesToSet) { // TODO change order of if statements?
        if (valueToSet == arg1) {
            assert (arg1->getType() == OSRGenInfo->arg1->getType());
            M->registerOneToOneValue(OSRGenInfo->arg1, arg1);
        } else if (valueToSet == arg2) {
            llvm::Type *oldMode = OSRGenInfo->arg2->getType();
            llvm::Type *newMode = arg2->getType();
            if (oldMode != newMode) {
                std::cerr << "OLD MODE: " << JITCompiler::LLVMTypeToString(oldMode) << std::endl;
                std::cerr << "NEW MODE: " << JITCompiler::LLVMTypeToString(newMode) << std::endl;
                throw CompError("Type mismatch for LLVM output argument!");
            } else {
                M->registerOneToOneValue(OSRGenInfo->arg2, arg2);
            }

        } else if (valueToSet == environment) {
            assert(OSRGenInfo->environment != nullptr); // TODO: can the environment be missing? can I still create it?
            M->registerOneToOneValue(OSRGenInfo->environment, environment);
        } else {
            // inspect VarMap for the new function
            JITCompiler::Value* newIIRVal = nullptr;
            SymbolExpr* newSym = nullptr;
            for (IIRVarMap::iterator it = optInfo->varMap->begin(),
                    end = optInfo->varMap->end(); it != end; ++it) {
                JITCompiler::Value* jitVal = it->second;
                if (valueToSet == jitVal->pValue) {
                    newIIRVal = jitVal;
                    newSym = it->first;
                    break;
                }
            }
            if (newIIRVal == nullptr) {
                if (llvm::AllocaInst* alloca = llvm::dyn_cast<llvm::AllocaInst>(valueToSet)) {
                    allocasToAdd.push_back(alloca);
                    if (hasNoPrevUses(valueToSet, predecessors)) {
                        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
                            std::cerr << "Reconstructing alloca never referenced in predecessor blocks" << std::endl;
                        }
                    } else if (inLoop) {
                        predecessors.erase(newBlock);
                        if (hasNoPrevUses(valueToSet, predecessors)) {
                            if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
                                std::cerr << "Reconstructing alloca accessed (bitcast?) in OSR dest block" << std::endl;
                            }
                        } else {
                            // TODO: liveness analysis info ensures that I will be writing the values using it before reading them?!?
                            std::cerr << "WARNING: reconstructing alloca instruction referenced in a predecessor block" << std::endl;
                            valueToSet->dump();
                        }
                        predecessors.insert(newBlock);
                    } else {
                        // TODO: liveness analysis info ensures that I will be writing the values using it before reading them?!?
                        std::cerr << "WARNING: reconstructing alloca instruction referenced in a predecessor block" << std::endl;
                        valueToSet->dump();
                    }
                } else {
                    std::cerr << "FATAL ERROR - missing information for value:" << std::endl;
                    valueToSet->dump();
                    assert(false);
                }
            } else {
                // look for a match in the VarMap of the old function
                JITCompiler::Value* oldIIRVal = nullptr;
                for (IIRVarMap::iterator it = infoAtOldIIR->varMap->begin(),
                        end = infoAtOldIIR->varMap->end(); it != end; ++it) {
                    if (it->first == newSym) { // pointer comparison for SymbolExpr (see class)
                        oldIIRVal = it->second;
                        break;
                    }
                }
                assert(oldIIRVal != nullptr);

                SymToIIRValPairMap[newSym] = JITValPair(newIIRVal, oldIIRVal);
            }
        }
    }

    // add compensation code for alloca instructions
    StateMap::BlockPairInfo& bpInfo = M->getOrCreateMapBlockPairInfo(blockPair);
    StateMap::ValueInfoMap &valInfoMap = bpInfo.valueInfoMap;
    for (llvm::AllocaInst* destAlloca: allocasToAdd) {
        llvm::Value* cloneAlloca = destAlloca->clone();
        if (destAlloca->hasName()) cloneAlloca->setName(destAlloca->getName());
        StateMap::CompCode* compCode = new StateMap::CompCode();
        compCode->args = nullptr;
        compCode->code = new llvm::SmallVector<llvm::Value*, 1>;
        compCode->code->push_back(cloneAlloca);
        compCode->value = cloneAlloca;

        StateMap::ValueInfo* valInfo = new StateMap::ValueInfo(compCode);
        llvm::Value* destVal = llvm::cast<llvm::Value>(destAlloca);
        valInfoMap.insert(std::pair<llvm::Value*, StateMap::ValueInfo*>(destVal, valInfo));
    }

    // now we can process IIR variables [and add compensation code where needed]
    for (std::map<SymbolExpr*, JITValPair>::iterator it = SymToIIRValPairMap.begin(),
            end = SymToIIRValPairMap.end(); it != end; ++it) {
        SymbolExpr* sym = it->first;
        JITValPair &valPair = it->second;
        JITCompiler::Value* newVal = valPair.first;
        JITCompiler::Value* oldVal = valPair.second;

        if (newVal->objType != oldVal->objType) {
            std::cerr << "Type conversion required for variable " << sym->getSymName() << std::endl;
            generateTypeConversionCompCode(sym, valPair, M, bpInfo, modForNewFun, oldEnv);
        } else {
            M->registerOneToOneValue(oldVal->pValue, newVal->pValue);
        }
    }

    M->registerCorrespondingBlock(origBlock, newBlock);

    // at this point we can generate the continuation function
    std::string OSRDestFunName = (newFunc->getName().str()).append("DestOSR");
    std::vector<llvm::Value*> *passedValues = OSRGenInfo->passedValues;
    llvm::Function* OSRDestFun = OSRLibrary::generateOSRDestFun(*JITCompiler::s_Context ,*origFunc,
            *newFunc, blockPair, *passedValues, *M, &OSRDestFunName);

    // insert continuation function into LLVM Module
    modForNewFun->getFunctionList().push_back(OSRDestFun);

    return std::pair<StateMap*, llvm::Function*>(M, OSRDestFun);
}

OSRFeval::OptimizedFunPair OSRFeval::generateIIRFunc(ProgFunction* orig, Function* calledFunc,
        OSRFeval::FevalInfoForOSRGen* genInfo) {
    FevalAnalysisInfo* analysis = const_cast<FevalAnalysisInfo*>(genInfo->pCompVersion->pFevalInfo);
    ParamExpr* pExpr = genInfo->fevalInfoForOSR->pParamExpr;

    // determine which symbols we should manipulate
    SymbolExpr* pSymForFeval = const_cast<SymbolExpr*>((const SymbolExpr*)pExpr->getArgument(0));
    SymbolExpr* pSymToCall = SymbolExpr::getSymbol(calledFunc->getFuncName());

    if (ConfigManager::s_veryVerboseVar) {
        std::cerr << "Symbol to replace in feval: " << pSymForFeval->getSymName() << std::endl;
        std::cerr << "Symbol to call directly: " << pSymToCall->getSymName() << std::endl;
    }

    // determine which statements should be replaced
    std::vector<FevalAnalysisInfo::FevalCallInfo*> &vecFevalCallInfo = analysis->ConstantFirstArg[pSymForFeval];
    std::set<AssignStmt*> assignStmts;
    for (FevalAnalysisInfo::FevalCallInfo* info: vecFevalCallInfo) {
        assignStmts.insert(info->assStmt);
    }

    /* Clone IIR function and update it */
    ProgFunction* newFun = orig->copyWithCurrentBody();
    newFun->setFuncName("OSR_" + orig->getFuncName());

    std::map<AssignStmt*, AssignStmt*> mapNewToOldAssignSmts;
    std::map<ParamExpr*, ParamExpr*> mapOldToNewParamExprs;

    parseClonedFunForIIRMapping(orig->getCurrentBody(), newFun->getCurrentBody(), assignStmts, mapNewToOldAssignSmts);
    assert(assignStmts.empty());

    for (std::map<AssignStmt*, AssignStmt*>::iterator it = mapNewToOldAssignSmts.begin(),
            end = mapNewToOldAssignSmts.end(); it != end; ++it) {
        AssignStmt* pNewStmt = it->first;
        AssignStmt* pOldStmt = it->second;

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

        mapOldToNewParamExprs.insert(std::pair<ParamExpr*, ParamExpr*>(
                                (ParamExpr*)pOldStmt->getRightExpr(), pExpr));

        if (ConfigManager::s_veryVerboseVar) {
            std::cerr << "New expr: " << pExpr->toString() << std::endl;
        }
    }

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << orig->toString();
        std::cerr << newFun->toString() << std::endl;
    }

    return OptimizedFunPair(newFun, std::move(mapOldToNewParamExprs));
}

// code adapted from JITCompiler::compileFunction()
std::pair<llvm::Function*, OSRFeval::CompPair> OSRFeval::generateIRforFunction(ProgFunction* pFunction,
        JITCompiler::CompFunction* pOldCompFunc, JITCompiler::CompVersion* pOldCompVersion,
        std::map<ParamExpr*, ParamExpr*> &optimizedParamExprMap) {
    // initial MCJITHelper integration (see also end of function)
    llvm::Module* MCJITModule = JITCompiler::s_JITHelper->generateFreshModule();
    llvm::Module* prevModule = JITCompiler::s_MCJITModuleInUse;
    JITCompiler::s_MCJITModuleInUse = MCJITModule;

    // get the input argument types
    TypeSetString &argTypeStr = pOldCompVersion->inArgTypes;

    // set the local environment for the function
    ProgFunction::setLocalEnv(pFunction, ProgFunction::getLocalEnv(pOldCompFunc->pProgFunc)); // TODO copy the env??

    JITCompiler::CompFunction tmpCompFunction; /* TODO: avoid duplicates */
    tmpCompFunction.pProgFunc = pFunction;
    tmpCompFunction.pFuncBody = pFunction->getCurrentBody(); // already transformed!

    JITCompiler::s_functionMap.insert(std::pair<ProgFunction*, JITCompiler::CompFunction>(pFunction, tmpCompFunction));

    // get compFunction and compVersion, then store the input argument types
    JITCompiler::CompFunction &compFunction = JITCompiler::s_functionMap.find(pFunction)->second;
    JITCompiler::CompVersion &compVersion = compFunction.versions[argTypeStr];
    compVersion.inArgTypes = argTypeStr;

    // perform analyses on IIR
    compVersion.pReachDefInfo = (const ReachDefInfo*)AnalysisManager::requestInfo(
	&computeReachDefs, pFunction, compFunction.pFuncBody, compVersion.inArgTypes);
    compVersion.pLiveVarInfo = (const LiveVarInfo*)AnalysisManager::requestInfo(&computeLiveVars,
	pFunction, compFunction.pFuncBody, compVersion.inArgTypes);
    compVersion.pTypeInferInfo = (const TypeInferInfo*)AnalysisManager::requestInfo(&computeTypeInfo,
	pFunction, compFunction.pFuncBody, compVersion.inArgTypes);
    compVersion.pMetricsInfo = (const MetricsInfo*)AnalysisManager::requestInfo(&computeMetrics,
	pFunction, compFunction.pFuncBody, compVersion.inArgTypes);
    compVersion.pBoundsCheckInfo = (const BoundsCheckInfo*)AnalysisManager::requestInfo(&computeBoundsCheck,
	pFunction, compFunction.pFuncBody, compVersion.inArgTypes);
    if (JITCompiler::s_jitCopyEnableVar) {
	compVersion.pArrayCopyInfo = (const ArrayCopyAnalysisInfo*)AnalysisManager::requestInfo(
            &ArrayCopyElim::computeArrayCopyElim, pFunction, compFunction.pFuncBody, compVersion.inArgTypes);
    }

    // hack: force the parameter types at exit to be the same as in the original function!
    TypeInfoMap::const_iterator oldTypeInfoItr = pOldCompVersion->pTypeInferInfo->postTypeMap.find(pOldCompFunc->pFuncBody);
    TypeInferInfo* newTypeInferInfo = const_cast<TypeInferInfo*>(compVersion.pTypeInferInfo);
    newTypeInferInfo->postTypeMap[compFunction.pFuncBody] = oldTypeInfoItr->second;

    // feval analysis should be treated separately as we have to track optimized expressions
    FevalAnalysisInfo* fevalAnalysisInfo = const_cast<FevalAnalysisInfo*>( (const FevalAnalysisInfo*)
        AnalysisManager::requestInfo(&computeFevalInfo, pFunction, compFunction.pFuncBody, compVersion.inArgTypes));
    for (std::pair<ParamExpr* const, ParamExpr*> &pair: optimizedParamExprMap) {
        fevalAnalysisInfo->OptimizedParamExprs.insert(pair.second);
    }
    fevalAnalysisInfo->containsParamExprsToTrack = true;
    compVersion.pFevalInfo = (const FevalAnalysisInfo*)fevalAnalysisInfo;


    if (ConfigManager::s_veryVerboseVar || ConfigManager::s_verboseVar) {
	std::cerr << "Analysis process complete" << std::endl;
    }

    // generate IR code for the function
    llvm::Function* pFuncObj = JITCompiler::compileFunctionGenerateIR(pFunction, compFunction,
                                    compVersion, argTypeStr, MCJITModule);

    /* TODO: feval optimization pass for remaining calls*/

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "--> Generated optimized IR code <---" << std::endl;
        pFuncObj->dump();
    }

    // initial MCJITHelper integration (see also beginning of function)
    JITCompiler::s_MCJITModuleInUse = prevModule;

    return std::pair<llvm::Function*, CompPair>(pFuncObj, CompPair(&compFunction, &compVersion));
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

void OSRFeval::compCodeFromUnknownType(JITCompiler::Value* oldVal, JITCompiler::Value* newVal,
        StateMap* M, StateMap::BlockPairInfo& bpInfo, llvm::Module* currModule) {
    llvm::Type* newMode = newVal->pValue->getType();
    llvm::Type* oldMode = oldVal->pValue->getType();
    DataObject::Type newType = newVal->objType;

    llvm::LLVMContext &Context = *JITCompiler::s_Context;

    assert(oldMode == llvm::Type::getInt8PtrTy(Context));
    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "OLD MODE: " << JITCompiler::LLVMTypeToString(oldMode) << std::endl;
        std::cerr << "NEW MODE: " << JITCompiler::LLVMTypeToString(newMode) << std::endl;
    }

    if (newMode == oldMode) {
        std::cerr << "WARNING: requested a cast from UNKNOWN to "
                << DataObject::getTypeName(newType) << " with same " <<
                JITCompiler::LLVMTypeToString(oldMode) << " IR type!" << std::endl;
        M->registerOneToOneValue(oldVal->pValue, newVal->pValue); // no compensation code required!
        return;
    }

    StateMap::CompCode* compCode = new StateMap::CompCode();
    bool compCodeGenerated = false;

    if (newMode == llvm::Type::getDoubleTy(Context)) {
        if (newType == DataObject::MATRIX_F64) {
            if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
                std::cerr << "Performing cast to UNKNOWN (i8*) to f64 matrix (double*)" << std::endl;
            }
            const JITCompiler::NativeFunc& nativeFunc = JITCompiler::s_nativeMap[(void*)MatrixF64Obj::getScalarVal];
            llvm::Function* functionToCall = JITCompiler::getLLVMFunctionToCall(nativeFunc.pLLVMFunc, currModule);
            JITCompiler::LLVMValueVector args(1, oldVal->pValue);
            llvm::CallInst* compVal = llvm::CallInst::Create(functionToCall, args, "ccUKtoMF64");

            compCode->args = new llvm::SmallVector<llvm::Value*, 1>;
            compCode->args->push_back(oldVal->pValue);
            compCode->code = new llvm::SmallVector<llvm::Value*, 1>;
            compCode->code->push_back(compVal);
            compCode->value = compVal;

            compCodeGenerated = true;
        }
    }

    if (compCodeGenerated) {
        StateMap::ValueInfo* valInfo = new StateMap::ValueInfo(compCode);
        llvm::Value* destVal = newVal->pValue;
        bpInfo.valueInfoMap.insert(std::pair<llvm::Value*, StateMap::ValueInfo*>(destVal, valInfo));
    } else {
        std::cerr << "Sorry, for the time being I can only convert a limited number of type combinations" << std::endl;
        assert(false);
    }

}

void OSRFeval::compCodeForMissingVal(SymbolExpr* sym, OSRFeval::JITValPair& valPair, StateMap* M,
        StateMap::BlockPairInfo& bpInfo, llvm::Module* currModule, llvm::Value* oldEnv) {

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "I will ask the environment to give me the object!" << std::endl;
    }

    StateMap::CompCode* compCode = new StateMap::CompCode();

    compCode->args = new llvm::SmallVector<llvm::Value*, 1>;
    compCode->args->push_back(oldEnv);

    JITCompiler::LLVMValueVector lookupArgs;
    lookupArgs.push_back(oldEnv);
    lookupArgs.push_back(JITCompiler::createPtrConst(sym));

    const JITCompiler::NativeFunc& envLookupNativeFunc = JITCompiler::s_nativeMap[(void*)Environment::lookup];
    llvm::Function* envLookupFun = JITCompiler::getLLVMFunctionToCall(envLookupNativeFunc.pLLVMFunc, currModule);
    llvm::CallInst* envResult = llvm::CallInst::Create(envLookupFun, lookupArgs, "envLookupFor"+sym->getSymName()); // i8*

    JITCompiler::Value* newVal = valPair.first;
    llvm::Type* newType = newVal->pValue->getType();
    if (newType->isPointerTy()) {
        llvm::PointerType *newPtrType = llvm::cast<llvm::PointerType>(newType);
        assert (newPtrType->isPointerTy());

        compCode->code = new llvm::SmallVector<llvm::Value*, 1>;
        compCode->code->push_back(envResult);
        compCode->value = envResult;

        StateMap::ValueInfo* valInfo = new StateMap::ValueInfo(compCode);
        llvm::Value* destVal = newVal->pValue;
        bpInfo.valueInfoMap.insert(std::pair<llvm::Value*, StateMap::ValueInfo*>(destVal, valInfo));
    } else {
        throw CompError("Sorry, I can only convert to pointers when performing env lookup!");
    }
}


void OSRFeval::generateTypeConversionCompCode(SymbolExpr* sym, OSRFeval::JITValPair &valPair,
        StateMap* M, StateMap::BlockPairInfo& bpInfo, llvm::Module* currModule, llvm::Value* oldEnv) {
    JITCompiler::Value* newVal = valPair.first;
    JITCompiler::Value* oldVal = valPair.second;
    DataObject::Type oldType = oldVal->objType, newType = newVal->objType;
    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "--> new type: " << DataObject::getTypeName(newType) << std::endl;
        std::cerr << "    corresponding IR: "; newVal->pValue->dump();
        std::cerr << "--> old type: " << DataObject::getTypeName(oldType) << std::endl;
    }

    if (oldVal->pValue == nullptr) {
        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
            std::cerr << "------> WARNING: NOT AN IR VALUE! <------" << std::endl;
        }
        compCodeForMissingVal(sym, valPair, M, bpInfo, currModule, oldEnv);
        return;
    }

    if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
        std::cerr << "    corresponding IR: "; oldVal->pValue->dump();
    }


    switch(oldType) {
        case DataObject::UNKNOWN: {
            compCodeFromUnknownType(oldVal, newVal, M, bpInfo, currModule);
        } break;

        default: {
            throw CompError("Sorry, I can only convert UNKNOWN object types for now!");
        }
    }
}