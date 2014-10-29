CLANG   = clang
CLANG_CFLAGS =  -cc1 -emit-llvm-bc -x cl -O2 -fno-builtin -nobuiltininc
CLANG_CFLAGS += -Fvisibility=protected -cl-std=CL1.2 -ffp-contract=off 
CLANG_CFLAGS += -I../../include 

CL_FILES = $(wildcard *.cl)
BYTECODE := ${CL_FILES:.cl=.bc}

all: builtins.lib

builtins.lib: $(BYTECODE)
	@echo $@ Linking bytecode modules 
	llvm-link -o $@ $^

%.bc: %.cl
	@echo $< Parsing
	@$(CLANG) $(CLANG_CFLAGS) $< -o $@

%.ll: %.bc
	@echo $< Disassembling
	llvm-dis $<

clean:
	@rm -rf *.bc *.ll
