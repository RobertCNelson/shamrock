# If building for ARM target host then set appropriate clang target
# Needs to match what's used when using clang to build the kernel
# See compiler.cpp 
if (HAWKING_BUILD)
   set(HOST_TARGET -target spir-unknown-unknown-unknown)
endif()

# If Shamrock build, then we use the builtins.lib built in ../builtins
if (SHAMROCK_BUILD)
add_custom_command(
    OUTPUT stdlib.c.bc.embed.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/embed.py
            ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc.embed.h
            ${CMAKE_CURRENT_BINARY_DIR}/../builtins/builtins.lib
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/embed.py
            ${CMAKE_CURRENT_BINARY_DIR}/../builtins/builtins.lib)

add_custom_target(generate_stdlib_c DEPENDS
                  ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc.embed.h)
# otherwise, this stdlib.c is still being used (but is empty)
else (SHAMROCK_BUILD)
    set(CUSTOM_COMMAND
    ${CLANG_EXECUTABLE} -c -emit-llvm -x cl -O2 ${HOST_TARGET} -nostdinc -fno-builtin)

add_custom_command(
    OUTPUT stdlib.c.bc
    COMMAND ${CUSTOM_COMMAND}
            -I${OCL_BUILTINS_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/stdlib.c
            -I${CMAKE_CURRENT_BINARY_DIR}
            -o ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stdlib.c
            ${CMAKE_CURRENT_BINARY_DIR}/stdlib_impl.h)

add_custom_command(
    OUTPUT stdlib.c.bc.embed.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/embed.py
            ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc.embed.h
            ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/embed.py
            ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc)

add_custom_target(generate_stdlib_c DEPENDS
                  ${CMAKE_CURRENT_BINARY_DIR}/stdlib.c.bc.embed.h)

add_custom_command(
    OUTPUT builtins_def.h stdlib_def.h builtins_impl.h stdlib_impl.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/builtins.py
            ${CMAKE_CURRENT_SOURCE_DIR}/builtins.def
            ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/builtins.py
            ${CMAKE_CURRENT_SOURCE_DIR}/builtins.def)

add_custom_target(generate_builtins DEPENDS
                  ${CMAKE_CURRENT_BINARY_DIR}/builtins_def.h
                  ${CMAKE_CURRENT_BINARY_DIR}/builtins_impl.h
                  ${CMAKE_CURRENT_BINARY_DIR}/stdlib_def.h
                  ${CMAKE_CURRENT_BINARY_DIR}/stdlib_impl.h)
endif(SHAMROCK_BUILD)
