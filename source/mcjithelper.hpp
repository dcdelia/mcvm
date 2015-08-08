/* ===============================================================
 * Helper class to use LLVM MCJIT compiler in McVM. Some of this
 * code is adapted from TinyVM (http://github.com/dcdelia/tinyvm)
 *
 * (C) Daniele Cono D'Elia, Sapienza University of Rome, 2015.
 * =============================================================== */

#ifndef MCJITHELPER_HPP
#define	MCJITHELPER_HPP

#include <llvm/ADT/StringRef.h>
#include <llvm/ADT/Twine.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/ExecutionEngine/SectionMemoryManager.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/TargetSelect.h>

#include <map>
#include <iostream>
#include <string>

class MCJITHelper {
public:
    MCJITHelper(std::string sessionName, llvm::LLVMContext* Context, bool verbose = false) :
        pContext(Context), verbose(verbose), modulePrefix(sessionName), moduleIndex(0) {

        std::unique_ptr<llvm::Module> InitialModule = llvm::make_unique<llvm::Module>(modulePrefix, *pContext);
        lastModule = InitialModule.get();
        std::string ErrStr;
        ExecEngine = llvm::EngineBuilder(std::move(InitialModule))
                            .setEngineKind(llvm::EngineKind::JIT)
                            .setErrorStr(&ErrStr)
                            .setOptLevel(llvm::CodeGenOpt::None)
                            .create();
        if (verbose) {
            std::cerr << "[MCJITHelper] Successfully started!" << std::endl;
        }
    }

    ~MCJITHelper();

    llvm::Module            *lastModule;
    llvm::ExecutionEngine   *ExecEngine;
    llvm::LLVMContext       *pContext;

    llvm::Module*   generateFreshModule();
    void            registerNativeFunction(const std::string &Name, void* ptr);

private:
    bool            verbose;
    std::map<const std::string, void*> nativeMap;
    std::string     modulePrefix;
    unsigned int    moduleIndex;
};

#endif	/* MCJITHELPER_HPP */

