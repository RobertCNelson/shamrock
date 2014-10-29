Shamrock: an OpenCL implementation based on clover

This is a continuation of the clover OpenCL project:
	http://people.freedesktop.org/~steckdenis/clover

based on the contributions from Texas Instruments for Keystone II DSP device:
        git.ti.com/opencl

and adding contributions from Linaro for ARM CPU-only support.

Prereqs
=======
The following packages need to be installed on your system prior to build:
 
gcc 4.8 (for building llvm)
cmake
check
libboost-all-dev
libtinfo-dev
mesa-common-dev
python 2.6+, and not greater or equal to v 3.0.

BUILD
=====

LLVM Configuration:
-------------------

This was tested using LLVM 3.5.0 stable release from:
http://llvm.org/releases/download.html

Note: LLVM must be configured and built with certain options to link with shamrock for
ARM.

The following creates a release build for ARM, with LLVM installed 
into /opt/llvm:

% CC=gcc CXX=g++ ./configure --prefix=/opt/llvm --enable-jit --enable-targets=arm --enable-optimized --enable-assertions --with-float=hard --with-abi=aapcs-vfp
% make -j4 REQUIRES_RTTI=1
% sudo make -j4 install

See:  http://llvm.org/releases/3.5.0/docs/HowToBuildOnARM.html  for updates.

Shamrock Build:
---------------

Current Branch: Khronos_conformance

Usage: cmake <project_src_dir> <optional_defines>*
  where <optional_defines*> are:
    -DPROJECT=shamrock | shannon | hawking
    -DLLVM_CONFIG_EXECUTABLE=<path to private llvm-config version>
Note PROJECT=shamrock is default.

The best way to compile is to use an out of src build, eg for a Debug build, 
and custom LLVM:

% mkdir shamrock_build
% cd shamrock_build
% cmake -DLLVM_CONFIG_EXECUTABLE=/opt/llvm/bin/llvm-config -DCMAKE_BUILD_TYPE=Debug <path_to>/shamrock
% make
% sudo make install

If your Clang is installed to a different location than LLVM,
then define CLANG_INCLUDE_DIR and CLANG_LIB_DIR on the cmake cmd line:

  -DCLANG_INCLUDE_DIR=/opt/clang/include -DCLANG_LIB_DIR=/opt/clang/lib


SANITY TESTS
============

The build commands above will build some simple sanity tests.

% cd shamrock_build
% make test

Latest Results:
---------------

shamrock_build> make test
Running tests...
/usr/bin/ctest --force-new-ctest-process 
Test project /home/gpitney/shamrock_build
    Start 1: platform
1/8 Test #1: platform .........................   Passed    0.11 sec
    Start 2: device
2/8 Test #2: device ...........................   Passed    0.01 sec
    Start 3: context
3/8 Test #3: context ..........................   Passed    0.01 sec
    Start 4: commandqueue
4/8 Test #4: commandqueue .....................   Passed    1.03 sec
    Start 5: mem
5/8 Test #5: mem ..............................   Passed    0.01 sec
    Start 6: kernel
6/8 Test #6: kernel ...........................***Failed    0.90 sec
    Start 7: program
7/8 Test #7: program ..........................   Passed    2.17 sec
    Start 8: builtins
8/8 Test #8: builtins .........................   Passed    1.53 sec

88% tests passed, 1 tests failed out of 8

PIGLIT TESTS
============

If running PIGLIT OpenCL tests, to build for the OpenCL piglit binaries only:

% cd piglit
% cmake -DPIGLIT_BUILD_CL_TESTS=ON -DPIGLIT_BUILD_GL_TESTS=OFF \
  -DPIGLIT_USE_WAFFLE=OFF -DPIGLIT_USE_GLUT=OFF
% export PIGLIT_CL_VERSION=11
% make

To run OpenCL tests, results in results/all_cl/main

% piglit run tests/all_cl results/all_cl

DEBUGGING OpenCL Kernels:
=========================

1. printf:  A builtin function named "debug" maps to the printf symbol in the getBuiltin()
   callback function, allowing printf from OpenCL kernels.  Alternatively, this mechanism
   can be used to define aribraty functions to be called back from kernels.  

2. gdb:  Using the above getBuiltin() mechanism, a breakpoint can be placed in a callback
   function at kernel exit, then stepping back into the kernel via gdb, will allow
   debug of the kernel code (assembly level stepping).

