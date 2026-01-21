# Macro to check pkg-config modules and produce a full-path library list
macro(pkg_check_modules_fullpath_libs PREFIX MODULE_NAME)
	# Call pkg_check_modules (without REQUIRED to allow fallback)
	pkg_check_modules(${PREFIX} QUIET IMPORTED_TARGET ${MODULE_NAME})
	if(${PREFIX}_FOUND)
		set(${PREFIX}_FULLPATH_LIBRARIES "" PARENT_SCOPE)
		foreach(lib ${${PREFIX}_LIBRARIES})
			set(lib_path "")
			set(lib_prefix "")
			set(lib_ext "")
			if(WIN32)
				set(lib_ext ".lib")
			else()
				set(lib_prefix "lib")
				set(lib_ext ".a")
			endif()
			# Skip system libraries like 'm' on Windows
			if(WIN32 AND (lib STREQUAL "m"))
				continue()
			endif()
			# If lib already has a path, use as is
			if(EXISTS "${lib_prefix}${lib}${lib_ext}")
				set(lib_path "${lib_prefix}${lib}${lib_ext}")
			else()
				foreach(dir ${${PREFIX}_LIBRARY_DIRS})
					if(EXISTS "${dir}/${lib_prefix}${lib}${lib_ext}")
						set(lib_path "${dir}/${lib_prefix}${lib}${lib_ext}")
						break()
					endif()
				endforeach()
			endif()
			if(NOT lib_path STREQUAL "")
				list(APPEND ${PREFIX}_FULLPATH_LIBRARIES "${lib_path}")
			else()
				list(APPEND ${PREFIX}_FULLPATH_LIBRARIES "${lib}")
			endif()
		endforeach()
		set(${PREFIX}_FULLPATH_LIBRARIES "${${PREFIX}_FULLPATH_LIBRARIES}" PARENT_SCOPE)
		message(STATUS "Found ${MODULE_NAME}: ${${PREFIX}_VERSION} libs: ${${PREFIX}_FULLPATH_LIBRARIES}")
	else()
		set(${PREFIX}_FULLPATH_LIBRARIES "" PARENT_SCOPE)
		message(STATUS "${MODULE_NAME} not found, support will be disabled")
	endif()
endmacro()

# Find required packages
FIND_PACKAGE(PNG QUIET)
if(PNG_FOUND)
	INCLUDE_DIRECTORIES(${PNG_INCLUDE_DIRS})
	ADD_DEFINITIONS(${PNG_DEFINITIONS})
	SET(_USE_PNG TRUE CACHE INTERNAL "")
else()
	SET(PNG_LIBRARIES "")
endif()
FIND_PACKAGE(JPEG QUIET)
if(JPEG_FOUND)
	INCLUDE_DIRECTORIES(${JPEG_INCLUDE_DIR})
	ADD_DEFINITIONS(${JPEG_DEFINITIONS})
	SET(_USE_JPG TRUE CACHE INTERNAL "")
else()
	SET(JPEG_LIBRARIES "")
endif()
# Try to find libjxl via pkg-config first
FIND_PACKAGE(PkgConfig QUIET)
if(PkgConfig_FOUND)
	pkg_check_modules_fullpath_libs(JPEGXL libjxl)
endif()

# Fallback: if pkg-config didn't find it, try direct library search
if(NOT JPEGXL_FOUND)
	# Try to find libjxl directly
	find_path(JPEGXL_INCLUDE_DIR jxl/decode.h
		PATHS ${CMAKE_PREFIX_PATH}/include
		      $ENV{CONDA_PREFIX}/include
		      /usr/include
		      /usr/local/include
	)
	find_library(JPEGXL_LIBRARY
		NAMES jxl
		PATHS ${CMAKE_PREFIX_PATH}/lib
		      $ENV{CONDA_PREFIX}/lib
		      /usr/lib
		      /usr/local/lib
	)
	
	if(JPEGXL_INCLUDE_DIR AND JPEGXL_LIBRARY)
		set(JPEGXL_FOUND TRUE)
		set(JPEGXL_INCLUDE_DIRS ${JPEGXL_INCLUDE_DIR})
		set(JPEGXL_LIBRARIES ${JPEGXL_LIBRARY})
		include_directories(${JPEGXL_INCLUDE_DIRS})
		message(STATUS "Found libjxl via direct search: ${JPEGXL_LIBRARY}")
	endif()
endif()

if(JPEGXL_FOUND)
	SET(_USE_JXL TRUE CACHE INTERNAL "")
	# Ensure include directories are set (for direct search path)
	if(JPEGXL_INCLUDE_DIRS)
		include_directories(${JPEGXL_INCLUDE_DIRS})
	endif()
else()
	SET(JPEGXL_LIBRARIES "")
	message(STATUS "libjxl not found, JPEG-XL support will be disabled")
endif()
FIND_PACKAGE(TIFF QUIET)
if(TIFF_FOUND)
	INCLUDE_DIRECTORIES(${TIFF_INCLUDE_DIR})
	ADD_DEFINITIONS(${TIFF_DEFINITIONS})
	SET(_USE_TIFF TRUE CACHE INTERNAL "")
else()
	SET(TIFF_LIBRARIES "")
endif()

# List sources files
FILE(GLOB LIBRARY_FILES_C "*.cpp")
FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_library_with_type(IO "Libs" "" "${cxx_default}"
	${LIBRARY_FILES_C} ${LIBRARY_FILES_H}
)

# Manually set Common.h as the precompiled header
IF(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16.0)
	TARGET_PRECOMPILE_HEADERS(IO PRIVATE "Common.h")
endif()

# Link its dependencies
TARGET_LINK_LIBRARIES(IO PUBLIC Common ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${JPEGXL_LIBRARIES} ${TIFF_LIBRARIES} ${EXIV2_LIBS})

# Install
SET_TARGET_PROPERTIES(IO PROPERTIES
	PUBLIC_HEADER "${LIBRARY_FILES_H}")
INSTALL(TARGETS IO
	EXPORT OpenMVSTargets
	LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
	ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
	PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/IO")
