/* ===============================================================
 * Helper methods to deal with LLVM releases with no RTTI
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */
#include "LLVMUtils.hpp"

#include <map>
#include <utility>
#include <llvm/Transforms/Utils/ValueMapper.h>
#include <llvm/Transforms/Utils/Cloning.h>

LLVMUtils::ClonedFunc LLVMUtils::cloneFunction(llvm::Function* F) {
    llvm::ValueToValueMapTy VMap;

    llvm::Function* dup = llvm::CloneFunction(F, VMap, false, nullptr);
    dup->setName(F->getName());

    std::map<llvm::Value*, llvm::Value*> *map = new std::map<llvm::Value*, llvm::Value*>();

    for (llvm::ValueToValueMapTy::iterator it = VMap.begin(), end = VMap.end(); it != end; ++it) {
        llvm::Value* key = const_cast<llvm::Value*>(it->first);
        llvm::Value* value = it->second;
        (*map)[key] = value;
    }

    return ClonedFunc(dup, map);
}