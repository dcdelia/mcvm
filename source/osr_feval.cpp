#include "osr_feval.h"

#include "analysis_feval.h"
#include "jitcompiler.h"
#include "paramexpr.h"

#include <vector>
#include <utility>

OSRFeval::CompPairToOSRInfoMap OSRFeval::CompOSRInfoMap;
OSRFeval::CompPairToOSRPoints OSRFeval::CompOSRLocMap;

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

OSRFeval::LocForOSRPoints& OSRFeval::getLocationsForOSRPoints(JITCompiler::CompFunction* pCompFunction,
        JITCompiler::CompVersion* pCompVersion) {
    CompPair funPair(pCompFunction, pCompVersion);

    if (CompOSRLocMap.count(funPair) == 0) {
        LocForOSRPoints locs = computeLocationsForOSRPoints(const_cast<FevalInfo*>(pCompVersion->pFevalInfo));
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
