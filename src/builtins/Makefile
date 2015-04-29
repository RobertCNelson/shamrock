CLANG   = clang
CLANG_CFLAGS =  -cc1 -emit-llvm-bc -O2 -fno-builtin -nobuiltininc
CLANG_CFLAGS += -Fvisibility=protected -cl-std=CL1.2 -ffp-contract=off 
CLANG_CFLAGS += -I../../include -I/opt/llvm/lib/clang/3.6.0/include

CLANG_CL_FLAGS += $(CLANG_CFLAGS) -x cl

CL_FILES = $(wildcard *.cl)
C_FILES = $(wildcard *.c)
BYTECODE := ${CL_FILES:.cl=.bc}
BYTECODE_FROM_C := ${C_FILES:.c=.bc}

all: builtins.lib

builtins.lib: $(BYTECODE) $(BYTECODE_FROM_C)
	@echo $@ Linking bytecode modules 
	llvm-link -o $@ $^

%.bc: %.c
	@echo $< Parsing
	@$(CLANG) $(CLANG_CFLAGS) $< -o $@

%.bc: %.cl
	@echo $< Parsing
	@$(CLANG) $(CLANG_CL_FLAGS) $< -o $@

%.ll: %.bc
	@echo $< Disassembling
	llvm-dis $<

clean:
	@rm -rf *.bc *.ll
