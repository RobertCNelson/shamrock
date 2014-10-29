This directory (builtins) is intended to supercede src/runtime as a means
to provide a builtins library for OpenCL kernels.

Note: some of the files here do not compile due to an address space casting
error, and are suffixed *.cl.broken.

Files here were imported from the TI opencl_builtins private repository and
repurposed for CPU device (from DSP device).

This library appears to have been adapted from libclc.llvm.org.

The Makefile here is not used, but available for illustration purposes and
to allow disassmbly of the bc files for inspection.
