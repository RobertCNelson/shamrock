# Usage: cmake <project_src_dir> <optional_defines>*
#  where <optional_defines*> are:
#    -DPROJECT=shamrock | shannon | hawking
#    -DLLVM_CONFIG_EXECUTABLE=<path to private llvm-config version>
#    -DBUILD_TESTS=ON
# Note PROJECT=shamrock is default.
cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0022 OLD)

# Project Options:
OPTION(BUILD_TESTS "Set to ON to build minimal OpenCL tests" ON)
OPTION(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)

# For external builds, paths to dependent libraries/packages are assumed to
# be specified by environment variables.
# For internal development setup a default path for dependent libs/pkgs
if ("$ENV{DEFAULT_DEV_INSTALL_DIR}" STREQUAL "")
   set(DEFAULT_DEV_INSTALL_DIR /opt/ti)
else()
   set(DEFAULT_DEV_INSTALL_DIR $ENV{DEFAULT_DEV_INSTALL_DIR})
endif()

if (${PROJECT} MATCHES "shannon")
    project(opencl-dspc8681)
    SET(SHANNON_BUILD on)
    MESSAGE(STATUS "Build Target Is TI SHANNON")
    set(SDK "${DEFAULT_DEV_INSTALL_DIR}/desktop-linux-sdk_01_00_00_07")
elseif (${PROJECT} MATCHES "hawking")
    project(opencl-66ak2h)
    SET(HAWKING_BUILD on)
    MESSAGE(STATUS "Build Target Is TI HAWKING")
else()
    project(shamrock)
    SET(SHAMROCK_BUILD on)
    MESSAGE(STATUS "Project is shamrock")
endif()

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

# project version
SET(${PROJECT_NAME}_MAJOR_VERSION 0)
SET(${PROJECT_NAME}_MINOR_VERSION 8)
SET(${PROJECT_NAME}_PATCH_LEVEL 2)

# OpenCL platform version
SET(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_LEVEL})

# Set SONAME for OpenCL library
SET(${PROJECT_NAME}_SOVERSION ${${PROJECT_NAME}_MAJOR_VERSION})

if (NOT SHAMROCK_BUILD)
SET(CMAKE_INSTALL_PREFIX $ENV{HOME}/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION})
endif (NOT SHAMROCK_BUILD)
# Pending install changes
#if ("$ENV{TI_OCL_INSTALL_PREFIX}" STREQUAL "")
# set(CMAKE_INSTALL_PREFIX $ENV{HOME}/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION})
#else()
# set(CMAKE_INSTALL_PREFIX $ENV{TI_OCL_INSTALL_PREFIX}/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION})
#endif()

MESSAGE(STATUS "Install path is ${CMAKE_INSTALL_PREFIX}")

if (HAWKING_CROSS_COMPILE)
  set(CMAKE_SKIP_RPATH TRUE)
endif()

if (SHAMROCK_BUILD)
    ADD_DEFINITIONS("-DSHAMROCK_BUILD")
endif()
#ADD_DEFINITIONS("-DLOCK")

SET(CMAKE_VERBOSE_MAKEFILE TRUE)

# Find_Package() for LLVM/CLANG defined in CMAKE_MODULE_PATH
# Do not use system installed versions
Find_Package(LLVM REQUIRED)
Find_Package(Clang REQUIRED)

# OpenCL requires boost headers. If boost is installed to some directory
# other than /usr/include then define BOOST_INCLUDEDIR as below:
#set (BOOST_INCLUDEDIR path-to-boost-headers)
#include_directories(${BOOST_INCLUDEDIR})

SET (INCLUDE_INSTALL_DIR "include" CACHE PATH
  "The directory the headers are installed in")

# Set up install permissions for file, directories and binaries.
# !!! Note that these variables are used by the child cmake files so these
# variables must be set before the add_subdirectories() commands below
# File permissions (664)
set(OCL_FPERMS PERMISSIONS 
               OWNER_READ OWNER_WRITE 
	       GROUP_READ GROUP_WRITE 
	       WORLD_READ)

# Directory permissions (775). Files within the directories are set as above.
set(OCL_DPERMS FILE_${OCL_FPERMS} 
               DIRECTORY_PERMISSIONS 
	       OWNER_READ OWNER_WRITE OWNER_EXECUTE 
	       GROUP_READ GROUP_WRITE GROUP_EXECUTE 
	       WORLD_READ WORLD_EXECUTE)

# Binary permissions (775)
set(OCL_BPERMS PERMISSIONS 
               OWNER_READ OWNER_WRITE OWNER_EXECUTE 
	       GROUP_READ GROUP_WRITE GROUP_EXECUTE 
	       WORLD_READ WORLD_EXECUTE)


if (NOT OCL_EXAMPLES_DIR)
set(OCL_EXAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../opencl_examples)
endif()

if (NOT OCL_MONITOR_DIR)
set(OCL_MONITOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../opencl_monitor)
endif()

if (NOT OCL_BUILTINS_DIR)
set(OCL_BUILTINS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if (SHAMROCK_BUILD)
set(CLC_BUILTINS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/builtins)
endif()

add_subdirectory(src)
# add_subdirectory(util)
if (NOT SHAMROCK_BUILD)
add_subdirectory(clocl)
add_subdirectory(${OCL_EXAMPLES_DIR} ${CMAKE_CURRENT_BINARY_DIR}/examples)
add_subdirectory(${OCL_MONITOR_DIR} ${CMAKE_CURRENT_BINARY_DIR}/monitor)
add_subdirectory(${OCL_BUILTINS_DIR}/lib/dsp)
endif()

if (NOT HAWKING_CROSS_COMPILE)
IF (BUILD_TESTS)
    ENABLE_TESTING()
    Find_Package(Check REQUIRED)
    add_subdirectory(tests)
ENDIF (BUILD_TESTS)
endif()

# install OCL builtin and extension headers in opencl-headers package installation directory.
# TODO: We should require that the Khronos OpenCL headers have already been installed there.
if (SHAMROCK_BUILD)
#install(DIRECTORY include DESTINATION /usr ${OCL_DPERMS})
#install(DIRECTORY include/CL DESTINATION /usr/include ${OCL_DPERMS})
install(FILES include/clc.h DESTINATION /usr/include/CL )
install(FILES include/cpu.h DESTINATION /usr/include/CL )
install(FILES include/CL/cl_ext.h DESTINATION /usr/include/CL )
endif (SHAMROCK_BUILD)

# Following is TI product specific:
if (NOT SHAMROCK_BUILD)
# Create the top level directories with the desired install permissions
install(DIRECTORY DESTINATION .   ${OCL_DPERMS})
install(DIRECTORY DESTINATION bin ${OCL_DPERMS})
install(DIRECTORY DESTINATION lib ${OCL_DPERMS})

install(FILES opencl-manifest.pdf DESTINATION doc ${OCL_FPERMS})
install(DIRECTORY include DESTINATION . ${OCL_DPERMS})

install(FILES doc/opencl-1.1.pdf DESTINATION doc ${OCL_FPERMS})
install(FILES doc/opencl-cplusplus-1.1.pdf DESTINATION doc ${OCL_FPERMS})

if (SHANNON_BUILD)
 install(DIRECTORY DESTINATION init ${OCL_DPERMS})
 install(DIRECTORY scripts DESTINATION . ${OCL_DPERMS})
 install(DIRECTORY cmem DESTINATION . ${OCL_DPERMS})
 install(FILES init/init_dspc8681.out DESTINATION lib ${OCL_FPERMS})
 install(FILES init/init_dspc8682.out DESTINATION lib ${OCL_FPERMS})
 install(PROGRAMS bin/init_global_shared_mem DESTINATION bin ${OCL_BPERMS})
 install(PROGRAMS bin/oclenv DESTINATION bin ${OCL_BPERMS})
 install(FILES readme_shannon.txt DESTINATION . RENAME readme.txt ${OCL_FPERMS})
elseif(HAWKING_BUILD)
 install(DIRECTORY DESTINATION bin/arm ${OCL_DPERMS})
 install(DIRECTORY DESTINATION bin/x86 ${OCL_DPERMS})
 install(FILES doc/opencl_readme.pdf DESTINATION doc ${OCL_FPERMS})
endif()

set (CPACK_GENERATOR "DEB")
set (CPACK_DEBIAN_PACKAGE_MAINTAINER "TI")
set (CPACK_DEBIAN_PACKAGE_NAME ${PROJECT_NAME})
set (CPACK_DEBIAN_PACKAGE_ARCHITECTURE "all")

set (CPACK_PACKAGE_VERSION_MAJOR ${${PROJECT_NAME}_MAJOR_VERSION})
set (CPACK_PACKAGE_VERSION_MINOR ${${PROJECT_NAME}_MINOR_VERSION})
set (CPACK_PACKAGE_VERSION_PATCH ${${PROJECT_NAME}_PATCH_LEVEL})
set (CPACK_DEBIAN_PACKAGE_DEPENDS "mesa-common-dev (>=8.0.4-0), binutils-dev (>=2.22-6), libsqlite3-dev (>=3.7.9-2), libffi6 (>=3.0.11~rc1-5), zlib1g (>=1:1.2.3.4)")

#Set where dpkg will put the install
set (CPACK_INSTALL_PREFIX ${DEFAULT_DEV_INSTALL_DIR}/opencl/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION})
set (CPACK_SET_DESTDIR TRUE)


include(CPack)
endif(NOT SHAMROCK_BUILD)
