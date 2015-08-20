OUT = mcvm
OBJDIR = build
SRCDIR = source

# integration with novel OSR library written by D.C. D'Elia (github.com/dcdelia/tinyvm)
OSR_DIR = OSR
OSR_OBJDIR = $(OBJDIR)/OSR
OSR_INCLUDE =
OSR_CXXFLAGS =  $(OSR_INCLUDE) -O0 -g -Wall $(shell llvm-config --cxxflags) # no RTTI on LLVM "Release" builds

INCLUDE = -Ivendor/include -Ilib/include -I/usr/local/include # -I$(OSR_DIR)
CXXFLAGS =   $(INCLUDE) -Wall -g3 -std=c++11 -Wno-deprecated
CXXFLAGS += -D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -DMCVM_USE_LAPACKE -Wfatal-errors
#LLVMLIBS = $(shell llvm-config --libfiles)
LLVMLIBS = $(shell llvm-config  --ldflags --system-libs --libs core mcjit native)
LIBS = -pthread  -ldl -llapacke -lcblas
LIBS +=  vendor/lib/libgccpp.a  vendor/lib/libgc.a
CXX = g++
#CXX = clang++

_OBJS = analysis_arraycopy.o analysis_boundscheck.o analysis_copyplacement.o analysis_livevars.o \
		analysismanager.o analysis_metrics.o analysis_reachdefs.o analysis_typeinfer.o arrayobj.o \
		assignstmt.o binaryopexpr.o cellarrayexpr.o cellarrayobj.o cellindexexpr.o \
		chararrayobj.o client.o clientsocket.o configmanager.o dimvector.o endexpr.o \
		environment.o expressions.o exprstmt.o filesystem.o fnhandleexpr.o functions.o ifelsestmt.o \
		interpreter.o jitcompiler.o lambdaexpr.o loopstmts.o main.o matrixexpr.o matrixobjs.o matrixops.o objects.o \
		paramexpr.o parser.o plotting.o process.o profiling.o rangeexpr.o rangeobj.o runtimebase.o mcvmstdlib.o stmtsequence.o \
		switchstmt.o symbolexpr.o transform_endexpr.o transform_logic.o transform_loops.o transform_split.o transform_switch.o \
		typeinfer.o unaryopexpr.o utility.o xml.o mcjithelper.o analysis_feval.o osr_feval.o
OBJS = $(patsubst %,$(OBJDIR)/%,$(_OBJS))

_OSR_OBJS = Liveness.o OSRLibrary.o StateMap.o LLVMUtils.o
OSR_OBJS = $(patsubst %,$(OSR_OBJDIR)/%,$(_OSR_OBJS))

all: $(OUT)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OSR_OBJDIR):
	mkdir -p $(OSR_OBJDIR)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) -c $(INCLUDE) -o $@ $< $(CXXFLAGS)

$(OSR_OBJDIR)/%.o: $(OSR_DIR)/%.cpp | $(OSR_OBJDIR)
	$(CXX) -c $(INCLUDE) -o $@ $< $(OSR_CXXFLAGS)

$(OUT): $(OBJS) $(OSR_OBJS)
	$(CXX) $(OBJDIR)/*.o $(OSR_OBJDIR)/*.o $(LLVMLIBS) $(LIBS) -o $(OUT)

.PHONY: clean

clean:
	rm -f $(OBJDIR)/*.o $(OSROBJDIR)/*.o $(OUT)
