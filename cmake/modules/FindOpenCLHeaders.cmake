# FindOCLHeaders
#
# If successful, will define::
#
#   OCL_INCLUDE_DIR    - Set to where CL/cl.h was found
#   OCL_VERSION_STRING - Version of the OpenCL Headers "[Major].[minor]"
#

include(CheckSymbolExists)
find_path(OCL_INCLUDE_DIR NAMES CL/cl.h  PATHS /usr/include )

set(REQUIRED_VERSION "1_2")
if (${OCL_INCLUDE_DIR} STREQUAL "OCL_INCLUDE_DIR-NOTFOUND")
    message(STATUS "Please install opencl-headers package for version ${REQUIRED_VERSION}")
    message(FATAL_ERROR "ERROR: Unable to find installed OpenCL Headers")
endif()

set(CMAKE_REQUIRED_INCLUDES "${OpenCL_INCLUDE_DIR}")

CHECK_SYMBOL_EXISTS(CL_VERSION_${REQUIRED_VERSION}  "${OCL_INCLUDE_DIR}/CL/cl.h"
        OPENCL_VERSION_${VERSION})

if(OPENCL_VERSION_${VERSION})
    string(REPLACE "_" "." REQUIRED_VERSION "${REQUIRED_VERSION}")
    set(OCL_VERSION_STRING ${REQUIRED_VERSION} )
endif()

MESSAGE(STATUS "OpenCL Headers: Path: ${OCL_INCLUDE_DIR}; Version: ${OCL_VERSION_STRING}")

if ("${OCL_VERSION_STRING}" STREQUAL "" OR NOT "${OCL_VERSION_STRING}" STREQUAL "${REQUIRED_VERSION}")
    message(FATAL_ERROR  "ERROR: expected OpenCL Headers version ${REQUIRED_VERSION}")
endif()
