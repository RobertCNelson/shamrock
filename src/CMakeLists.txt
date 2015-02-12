
if (SHANNON_BUILD)
  SET (TARGET_INCLUDES
    ${PROJECT_SOURCE_DIR}/init
    ${SDK}/sdk
    ${SDK}/sdk/config
    ${SDK}/sdk/pciedrv
    ${SDK}/sdk/cmem
    ${SDK}/sdk/bufmgr
    ${SDK}/sdk/mailBox
    ${SDK}/sdk/dnldmgr )
# Cross-compiling needs additional paths to find target OS headers 
# and non-system headers found on the host (BOOST,GL)
elseif (HAWKING_CROSS_COMPILE)
  SET (TARGET_INCLUDES 
    ${CMAKE_FIND_ROOT_PATH} 
    ${HOST_USR_INCLUDE_PATH}  )
endif()


include_directories (
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${LLVM_INCLUDE_DIR}
    ${CLANG_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/src/core/dsp/ocl_load/DLOAD_API
    ${PROJECT_SOURCE_DIR}/src/core/dsp/ocl_load/DLOAD
    ${PROJECT_SOURCE_DIR}/src/llvmopencl
    ${TARGET_INCLUDES}
    )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FILE_OFFSET_BITS=64")

# Quiet warnings about deprecated OCL APIs:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

# bfd.h has a check to ensure that config.h is included
# We don't require config.h (autotools) so we bypass this check by defining
# PACKAGE, and PACKAGE_VERSION 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPACKAGE=${PROJECT_NAME} -DPACKAGE_VERSION=${${PROJECT_NAME}_VERSION}")

# Toggle below if wanting to build with debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-inline -g")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -fno-inline -g")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Temporary to work around hyperlink problem
set(CMAKE_C_FLAGS "${CMAKE_CFLAGS} -D__ARMv7 -DDEVICE_K2H")

if (SHANNON_BUILD)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDSPC868X")
endif()

configure_file(core/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/core/config.h)

set(COAL_SRC_FILES
    api/api_command.cpp
    api/api_device.cpp
    api/api_event.cpp
    api/api_kernel.cpp
    api/api_platform.cpp
    api/api_program.cpp
    api/api_context.cpp
    api/api_enqueue.cpp
    api/api_flush.cpp
    api/api_memory.cpp
    api/api_profiling.cpp
    api/api_sampler.cpp
    api/api_gl.cpp

    core/context.cpp
    core/commandqueue.cpp
    core/memobject.cpp
    core/events.cpp
    core/program.cpp
    core/compiler.cpp
    core/kernel.cpp
    core/sampler.cpp
    core/object.cpp
    core/platform.cpp
    core/icd.cpp
    core/util.cpp

    core/cpu/buffer.cpp
    core/cpu/device.cpp
    core/cpu/kernel.cpp
    core/cpu/program.cpp
    core/cpu/worker.cpp
    core/cpu/builtins.cpp
    core/cpu/sampler.cpp

    ${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib.h.embed.h
    ${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib.c.bc.embed.h
    ${CMAKE_CURRENT_BINARY_DIR}/runtime/builtins_impl.h
    ${CMAKE_CURRENT_BINARY_DIR}/runtime/builtins_def.h
    ${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib_impl.h
    ${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib_def.h
)

if (NOT SHAMROCK_BUILD)
list (APPEND COAL_SRC_FILES
    core/dsp/genfile_cache.cpp
    core/dsp/program.cpp
    core/dsp/wga.cpp
    core/dsp/driver.cpp
    core/dsp/buffer.cpp
    core/dsp/device.cpp
    core/dsp/kernel.cpp
    core/dsp/worker.cpp

    llvmopencl/AllocasToEntry.cc
    llvmopencl/BarrierBlock.cc
    llvmopencl/BarrierTailReplication.cc
    llvmopencl/BreakConstantGEPs.cpp
    llvmopencl/CanonicalizeBarriers.cc
    llvmopencl/Flatten.cc
    llvmopencl/GenerateHeader.cc
    llvmopencl/ImplicitLoopBarriers.cc
    llvmopencl/IsolateRegions.cc
    llvmopencl/Kernel.cc
    llvmopencl/LLVMUtils.cc
    llvmopencl/LoopBarriers.cc
    llvmopencl/ParallelRegion.cc
    llvmopencl/PHIsToAllocas.cc
    llvmopencl/TargetAddressSpaces.cc
    llvmopencl/VariableUniformityAnalysis.cc
    llvmopencl/WIVectorize.cc
    llvmopencl/Workgroup.cc
    llvmopencl/WorkItemAliasAnalysis.cc
    llvmopencl/WorkitemHandler.cc
    llvmopencl/WorkitemHandlerChooser.cc
    llvmopencl/WorkitemLoops.cc
    llvmopencl/WorkitemReplication.cc
)
endif(NOT SHAMROCK_BUILD)

if (SHAMROCK_BUILD)
add_subdirectory(builtins)
endif()
add_subdirectory(runtime)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-Bsymbolic")
add_library(OpenCL SHARED ${COAL_SRC_FILES})

set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib.h.embed.h
                            PROPERTIES GENERATED 1)
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib.c.bc.embed.h
                            PROPERTIES GENERATED 1)
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/runtime/builtins_impl.h
                            PROPERTIES GENERATED 1)
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/runtime/builtins_def.h
                            PROPERTIES GENERATED 1)
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib_impl.h
                            PROPERTIES GENERATED 1)
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/runtime/stdlib_def.h
                            PROPERTIES GENERATED 1)

add_dependencies(OpenCL generate_stdlib_c)

if (NOT SHAMROCK_BUILD)
add_dependencies(OpenCL generate_builtins)
add_dependencies(OpenCL oclload)
add_dependencies(OpenCL generate_dsp_builtins)
add_subdirectory(core/dsp/ocl_load)
else()
add_dependencies(generate_builtin_lib generate_bc_files)
add_dependencies(generate_stdlib_c generate_builtin_lib)
endif (NOT SHAMROCK_BUILD)

if (HAWKING_BUILD)
   add_dependencies(OpenCL arm_clocl)
endif()

if (HAWKING_CROSS_COMPILE OR SHANNON_BUILD)
   add_dependencies(OpenCL x86_clocl)
endif()

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET_TARGET_PROPERTIES(OpenCL PROPERTIES
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION ${${PROJECT_NAME}_SOVERSION}
)

set_source_files_properties(${COAL_SRC_FILES}
        PROPERTIES COMPILE_FLAGS ${LLVM_COMPILE_FLAGS})

set_target_properties(OpenCL PROPERTIES
  LINK_FLAGS "${LLVM_LDFLAGS}"
  LINK_INTERFACE_LIBRARIES "")

set (LIBS
        ${CLANG_LIBS}
        ${LLVM_LIBS_CORE}
        ${LLVM_LIBS_JIT}
        pthread
        rt
        dl
        z
        tinfo
        m
)

if (SHANNON_BUILD)
  LIST (APPEND LIBS
	${PROJECT_BINARY_DIR}/lib/liboclload.a
	${SDK}/sdk/pciedrv/lib/pciedrv.a`
	${SDK}/sdk/dnldmgr/lib/dnldmgr.a
	${SDK}/sdk/cmem/lib/cmem_drv.a
	${SDK}/sdk/bufmgr/lib/bufmgr.a
	${SDK}/sdk/mailBox/host/lib/mailBox.a
        pciaccess
	)
elseif(HAWKING_BUILD)
  LIST (APPEND LIBS 
	${PROJECT_BINARY_DIR}/lib/liboclload.a
	${LINUX_DEVKIT_ROOT}/usr/lib/libmpmmailbox.so
        ${LINUX_DEVKIT_ROOT}/usr/lib/libmpmtransport.so
        ${LINUX_DEVKIT_ROOT}/usr/lib/libmpmclient.so
        ${LINUX_DEVKIT_ROOT}/usr/lib/libticmem.so
        ${LINUX_DEVKIT_ROOT}/usr/lib/libkeystonemmap.so
	# We don't really depend on libhyplnk but link against it
	# to work around an mscsk issue. 
        ${LINUX_DEVKIT_ROOT}/usr/lib/libhyplnk_k2h.so)
endif()

if (NOT SHAMROCK_BUILD)
if (HAWKING_CROSS_COMPILE)
   SET(FFI_LIB     ${LINUX_DEVKIT_ROOT}/usr/lib/libffi.so.6)
   SET(BFD_LIB     ${LINUX_DEVKIT_ROOT}/usr/lib/libbfd.so)
   SET(SQLITE3_LIB ${LINUX_DEVKIT_ROOT}/usr/lib/libsqlite3.so.0)
else()
   find_library(FFI_LIB ffi)
   find_library(BFD_LIB bfd)
   find_library(SQLITE3_LIB sqlite3)
endif()

LIST (APPEND LIBS ${FFI_LIB} ${BFD_LIB} ${SQLITE3_LIB})
endif (NOT SHAMROCK_BUILD)

TARGET_LINK_LIBRARIES(OpenCL ${LIBS})
install(TARGETS OpenCL LIBRARY DESTINATION lib ${OCL_FPERMS})
