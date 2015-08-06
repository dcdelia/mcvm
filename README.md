# McVM for LLVM 3.6+

This repository is an initial effort to port the publicly available McVM virtual machine for MATLAB developed at McGill University to newer LLVM releases. Important design choices behind MCJIT - the new JIT compiler for LLVM - break the compatibility with code based on the old JIT, which has also been dropped since the 3.6 release. In this fork we hack the original code source code available from the [Sable](https://github.com/Sable/mcvm) repository to support MCJIT.

Our code has been built against LLVM 3.6.2, and should work fine in LLVM 3.6+ (and possibly 3.5+ as well). In order to build it, a few shared libraries are required: for most Ubuntu users, it should be enough to install *libatlas-base-dev*, *libblas-dev* and *liblapacke-dev*.

In order to compile the source, you should run the script *bootstrap.sh* to download and compile the [Boehm](http://www.hboehm.info/gc/) garbage collector used by McVM. Then you can simply run the command *make*.

For the sake of simplicity, in this repository we also provide a pre-compiled jar for the Natlab component (built from the sources from the Sable's [mclab-core](https://github.com/Sable/mclab-core) repository) required to construct the Abstract Syntax Tree for a MATLAB program.
