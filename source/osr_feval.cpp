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

#include <vector>
#include <utility>
#include <llvm/Analysis/Passes.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Value.h>
#include <llvm/PassManager.h>
//#include <llvm/Support/Casting.h>
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

    //currFunction->dump();

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
            std::cerr << "Available Value* from FevalInfoForOSR:" << std::endl;
            std::cerr << "arguments:" << std::endl;
            for (auto &pair: info->argsInArrayObj) {
                std::cerr << (void*)pair.first << " && " << (void*)pair.second << std::endl;
            }
            std::cerr << "" << std::endl;
            std::cerr << "call to interpreter: " << (void*)info->interpreterCallInst << std::endl;
            */

            std::cerr << "environment: " << (void*)pCompVersion->pEnvObject << std::endl;

            std::cerr << "Function arguments:" << std::endl;
            for (llvm::Function::arg_iterator argIt = currFunction->arg_begin(),
                    argEnd = currFunction->arg_end(); argIt != argEnd; ++argIt) {
                llvm::Value* arg = argIt;
                std::cerr << (void*)arg; arg->dump();
            }
        }

        FevalInfoForOSRGen* infoForOSRGen = new FevalInfoForOSRGen();
        infoForOSRGen->pCompFunction = pCompFunction;
        infoForOSRGen->pCompVersion = pCompVersion;
        infoForOSRGen->environment = pCompVersion->pEnvObject;
        assert(currFunction->getArgumentList().size() == 2);
        llvm::Function::arg_iterator argIt = currFunction->arg_begin();
        infoForOSRGen->arg1 = argIt++;
        infoForOSRGen->arg2 = argIt++;
        infoForOSRGen->passedValues = valuesToTransfer;

        // prepare data structures for insertOpenOSR
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

        OSRLibrary::OSRPair retOSRPair = OSRLibrary::insertOpenOSR(openOSRInfo, cond, nullptr, generator,
                                            true, "", valuesToTransfer);

        // insert stub into module
        currModule->getFunctionList().push_back(retOSRPair.second);

        if (ConfigManager::s_verboseVar || ConfigManager::s_veryVerboseVar) {
            retOSRPair.first->dump();
            retOSRPair.second->dump();
        }
    }

    return true;
}

void* OSRFeval::funGenerator(OSRLibrary::RawOpenOSRInfo *info, void* profDataAddr) {
    std::cerr << "Hi! I will generate optimized code on-the-fly :-)" << std::endl;
    assert(false);
    return nullptr;
}

OSRLibrary::OSRCond OSRFeval::generateDefaultOSRCond() {
    OSRLibrary::OSRCond cond;
    llvm::ConstantFP* one = llvm::ConstantFP::get(*JITCompiler::s_Context, llvm::APFloat(1.0));
    cond.push_back(new llvm::FCmpInst(llvm::CmpInst::FCMP_TRUE, one, one, "alwaysOSR"));

    return cond;
}
