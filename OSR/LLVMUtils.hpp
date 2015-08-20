/* ===============================================================
 * Helper methods to deal with LLVM releases with no RTTI
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */
#ifndef UTILS_HPP
#define	UTILS_HPP

#include <llvm/IR/Function.h>
#include <llvm/IR/Value.h>
#include <map>
#include <utility>

class LLVMUtils {
public:
    typedef std::pair<llvm::Function*, std::map<llvm::Value*, llvm::Value*>*> ClonedFunc;
    static ClonedFunc cloneFunction(llvm::Function* F);
};

#endif

