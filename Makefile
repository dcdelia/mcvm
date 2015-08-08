OUT = mcvm
OBJDIR = build
SRCDIR = source

INCLUDE = -Ivendor/include -Ilib/include -I/usr/local/include
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
		typeinfer.o unaryopexpr.o utility.o xml.o mcjithelper.o
OBJS = $(patsubst %,$(OBJDIR)/%,$(_OBJS))

all: $(OUT)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) -c $(INCLUDE) -o $@ $< $(CXXFLAGS)

$(OUT): $(OBJS)
	$(CXX) $(OBJDIR)/*.o $(LLVMLIBS) $(LIBS) -o $(OUT)

.PHONY: clean

clean:
	rm -f $(OBJDIR)/*.o $(OUT)
