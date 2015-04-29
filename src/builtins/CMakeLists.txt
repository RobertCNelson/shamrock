if (SHAMROCK_BUILD)

set(CUSTOM_COMMAND_C ${CLANG_EXECUTABLE} -cc1 -emit-llvm-bc -O2 -fno-builtin -nobuiltininc  -Fvisibility=protected -ffake-address-space-map -cl-std=CL1.2 -ffp-contract=off )

set(CUSTOM_COMMAND ${CUSTOM_COMMAND_C} -x cl )


FILE(GLOB CL_SOURCES ${CLC_BUILTINS_DIR}/*.cl)
FILE(GLOB C_SOURCES  ${CLC_BUILTINS_DIR}/*.c)
#MESSAGE(STATUS "C_SOURCES: ${C_SOURCES}" )

set(BC_SOURCES)
foreach(f ${CL_SOURCES})
    get_filename_component(fn ${f} NAME_WE)
    #MESSAGE(STATUS "CL_SOURCE: ${f}" )
    set(bc ${CMAKE_CURRENT_BINARY_DIR}/${fn}.bc)
    add_custom_command(OUTPUT ${bc} 
          COMMAND ${CUSTOM_COMMAND} 
            -I${OCL_BUILTINS_DIR}/include
            -o ${bc} ${f}
          DEPENDS ${f}
          COMMENT "Generating ${bc}")
    list(APPEND BC_SOURCES ${bc})
endforeach()


MESSAGE( STATUS "LLVM_LIB_DIR: ${LLVM_LIB_DIR}")
MESSAGE( STATUS "LLVM_VERSION: ${LLVM_VERSION}")
set(CLANG_INCLUDE_DIR "${LLVM_LIB_DIR}/clang/${LLVM_VERSION}/include" )
MESSAGE( STATUS "CLANG_INCLUDE_DIR: ${CLANG_INCLUDE_DIR}")

foreach(f ${C_SOURCES})
    get_filename_component(fn ${f} NAME_WE)
    #MESSAGE(STATUS "C_SOURCE: ${f}" )
    set(bc ${CMAKE_CURRENT_BINARY_DIR}/${fn}.bc)
    add_custom_command(OUTPUT ${bc}
          COMMAND ${CUSTOM_COMMAND_C}
            -I${OCL_BUILTINS_DIR}/include
            -I${CLANG_INCLUDE_DIR}
            -o ${bc} ${f}
          DEPENDS ${f}
          COMMENT "Generating ${bc}")
    list(APPEND BC_SOURCES ${bc})
endforeach()

#MESSAGE( STATUS "BC_SOURCES: ${BC_SOURCES}")

add_custom_target(generate_bc_files DEPENDS ${BC_SOURCES})

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/builtins.lib
          COMMAND llvm-link
            -o ${CMAKE_CURRENT_BINARY_DIR}/builtins.lib ${BC_SOURCES}
          DEPENDS ${BC_SOURCES} )

add_custom_target(generate_builtin_lib DEPENDS 
     ${CMAKE_CURRENT_BINARY_DIR}/builtins.lib)

endif(SHAMROCK_BUILD)
