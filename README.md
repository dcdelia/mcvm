# McVM for LLVM 3.6+

This repository is an initial effort to port the publicly available McVM virtual machine for MATLAB developed at McGill University to newer LLVM releases. Important design choices behind MCJIT - the new JIT compiler for LLVM - break the compatibility with (any VM based on) the old JIT, which has also been dropped from the most recent LLVM releases. In this fork we hack the original code source code available from the [Sable](https://github.com/Sable/mcvm) repository to support MCJIT.

Our code has been built against LLVM 3.6.2, and should work fine in LLVM 3.6+ (possibly 3.5+ as well). In order to build it, a few shared libraries are required (including *libatlas-base-dev* for Ubuntu users), as well as the Natlab jar component, which can be built from the Sable's [mclab-core](https://github.com/Sable/mclab-core) repository. Hopefully, more instructions will follow soon :-)
