# Macro to check pkg-config modules and produce a full-path library list
macro(pkg_check_modules_fullpath_libs PREFIX MODULE_NAME)
	# Call pkg_check_modules (without REQUIRED to allow fallback)
	pkg_check_modules(${PREFIX} QUIET IMPORTED_TARGET ${MODULE_NAME})
	if(${PREFIX}_FOUND)
		set(${PREFIX}_FULLPATH_LIBRARIES "" PARENT_SCOPE)
		foreach(lib ${${PREFIX}_LIBRARIES})
			set(lib_path "")
			set(lib_prefix "")
			if(WIN32)
				set(lib_exts ".lib")
			else()
				set(lib_prefix "lib")
				set(lib_exts ".so" ".a")
			endif()
			# Skip system libraries like 'm' on Windows
			if(WIN32 AND (lib STREQUAL "m"))
				continue()
			endif()
			# If lib already has a path, use as is
			if(EXISTS "${lib}")
				set(lib_path "${lib}")
			else()
				foreach(dir ${${PREFIX}_LIBRARY_DIRS})
					foreach(ext ${lib_exts})
						if(EXISTS "${dir}/${lib_prefix}${lib}${ext}")
							set(lib_path "${dir}/${lib_prefix}${lib}${ext}")
							break()
						endif()
					endforeach()
					if(NOT lib_path STREQUAL "")
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
		# Also set link directories for pkg-config found libraries
		if(${PREFIX}_LIBRARY_DIRS)
			link_directories(${${PREFIX}_LIBRARY_DIRS})
		endif()
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
	if(JPEGXL_FOUND AND JPEGXL_FULLPATH_LIBRARIES)
		# Use fullpath libraries from pkg-config
		set(JPEGXL_LIBRARIES ${JPEGXL_FULLPATH_LIBRARIES})
	endif()
endif()

# Fallback: if pkg-config didn't find it, try direct library search
if(NOT JPEGXL_FOUND)
	# Try to find libjxl directly
	# Collect all possible search paths
	set(JPEGXL_SEARCH_PATHS "")
	if(CMAKE_PREFIX_PATH)
		foreach(path ${CMAKE_PREFIX_PATH})
			list(APPEND JPEGXL_SEARCH_PATHS "${path}/lib" "${path}/lib64")
		endforeach()
	endif()
	if(DEFINED ENV{CONDA_PREFIX})
		list(APPEND JPEGXL_SEARCH_PATHS "$ENV{CONDA_PREFIX}/lib" "$ENV{CONDA_PREFIX}/lib64")
	endif()
	list(APPEND JPEGXL_SEARCH_PATHS "/usr/lib" "/usr/lib64" "/usr/local/lib" "/usr/local/lib64")
	
	find_path(JPEGXL_INCLUDE_DIR jxl/decode.h
		PATHS ${CMAKE_PREFIX_PATH}/include
		      $ENV{CONDA_PREFIX}/include
		      /usr/include
		      /usr/local/include
		NO_DEFAULT_PATH
	)
	if(NOT JPEGXL_INCLUDE_DIR)
		find_path(JPEGXL_INCLUDE_DIR jxl/decode.h)
	endif()
	
	find_library(JPEGXL_LIBRARY
		NAMES jxl
		PATHS ${JPEGXL_SEARCH_PATHS}
		NO_DEFAULT_PATH
	)
	if(NOT JPEGXL_LIBRARY)
		find_library(JPEGXL_LIBRARY NAMES jxl)
	endif()
	
	if(JPEGXL_INCLUDE_DIR AND JPEGXL_LIBRARY)
		set(JPEGXL_FOUND TRUE)
		set(JPEGXL_INCLUDE_DIRS ${JPEGXL_INCLUDE_DIR})
		set(JPEGXL_LIBRARIES ${JPEGXL_LIBRARY})
		# Get the directory containing the library for link_directories
		get_filename_component(JPEGXL_LIBRARY_DIR ${JPEGXL_LIBRARY} DIRECTORY)
		include_directories(${JPEGXL_INCLUDE_DIRS})
		link_directories(${JPEGXL_LIBRARY_DIR})
		message(STATUS "Found libjxl via direct search: ${JPEGXL_LIBRARY}")
		message(STATUS "JPEGXL library directory: ${JPEGXL_LIBRARY_DIR}")
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
