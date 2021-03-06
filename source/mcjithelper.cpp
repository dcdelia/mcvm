/* ===============================================================
 * Helper class to use LLVM MCJIT compiler in McVM. Some of this
 * code is adapted from TinyVM (http://github.com/dcdelia/tinyvm)
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */

#include "mcjithelper.hpp"

#include <llvm/ADT/StringRef.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/DynamicLibrary.h>

#include <cstring>
#include <iostream>
#include <map>
#include <string>

MCJITHelper::~MCJITHelper() {
    // for the time being we rely on llvm::shutdown() to avoid segfaults in McVM
}

llvm::Module* MCJITHelper::generateFreshModule() {
    std::string moduleName = modulePrefix + std::to_string(++moduleIndex);

    if (verbose) {
        std::cerr << "[MCJITHelper] Creating module " << moduleName << "..." << std::endl;
    }
    std::unique_ptr<llvm::Module> M = llvm::make_unique<llvm::Module>(moduleName, *pContext);
    llvm::Module* pM = M.get();

    ExecEngine->addModule(std::move(M));
    lastModule = pM;

    return pM;
}

void MCJITHelper::registerNativeFunction(const std::string &Name, void* ptr) {
    std::map<const std::string, void*>::iterator it = nativeMap.find(Name);
    if (it != nativeMap.end()) {
        char errorMsg[128];
        sprintf(errorMsg, "[MCJITHelper] Trying to register pointer %p for already-registered external function %s (@%p)",
                ptr, Name.c_str(), it->second);
        llvm::report_fatal_error(std::string(errorMsg));
    } else {
        nativeMap.insert(std::pair<const std::string, void*>(Name, ptr));
        llvm::sys::DynamicLibrary::AddSymbol(Name, ptr);
        if (verbose) {
            std::cerr << "[MCJITHelper] Function " << Name << " successfully registered" << std::endl;
        }
    }
}

