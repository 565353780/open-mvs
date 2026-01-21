###########################################################
#                  Find VCG Library
#----------------------------------------------------------

# Normalize VCG_ROOT to absolute path if provided
# Check both cache variable and regular variable
if(DEFINED VCG_ROOT)
    get_filename_component(VCG_ROOT "${VCG_ROOT}" ABSOLUTE)
elseif(DEFINED ENV{VCG_ROOT})
    get_filename_component(VCG_ROOT "$ENV{VCG_ROOT}" ABSOLUTE)
endif()

# Try to find vcg/complex/complex.h
# First try directly in VCG_ROOT (if provided) without PATH_SUFFIXES
if(VCG_ROOT)
    find_path(VCG_DIR "vcg/complex/complex.h"
        PATHS "${VCG_ROOT}"
        NO_DEFAULT_PATH
        DOC "Root directory of VCG library")
endif()

# If not found and VCG_ROOT was provided, try with PATH_SUFFIXES
if(NOT VCG_DIR AND VCG_ROOT)
    find_path(VCG_DIR "vcg/complex/complex.h"
        PATHS "${VCG_ROOT}"
        PATH_SUFFIXES "vcg" "include"
        NO_DEFAULT_PATH
        DOC "Root directory of VCG library")
endif()

# If still not found, try standard search paths
if(NOT VCG_DIR)
    find_path(VCG_DIR "vcg/complex/complex.h"
        HINTS "${VCG_ROOT}" "$ENV{VCG_ROOT}"
        PATHS "$ENV{PROGRAMFILES}" "$ENV{PROGRAMW6432}" "/usr" "/usr/local" "/usr/share" "/usr/local/share" "/usr/lib/x86_64-linux-gnu/cmake"
        PATH_SUFFIXES "vcg" "include"
        DOC "Root directory of VCG library")
endif()

##====================================================
## Include VCG library
##----------------------------------------------------
if(EXISTS "${VCG_DIR}" AND NOT "${VCG_DIR}" STREQUAL "")
	set(VCG_FOUND TRUE)
	set(VCG_INCLUDE_DIRS ${VCG_DIR})
	set(VCG_DIR "${VCG_DIR}" CACHE PATH "" FORCE)
	mark_as_advanced(VCG_DIR)
	set(VCG_INCLUDE_DIR ${VCG_DIR})

	message(STATUS "VCG ${VCG_VERSION} found (include: ${VCG_INCLUDE_DIRS})")
else()
	package_report_not_found(VCG "Please specify VCG directory using VCG_ROOT env. variable")
endif()
##====================================================
