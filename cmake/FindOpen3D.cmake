# Find Open3D library
# This module finds the Open3D library and sets the following variables:
# OPEN3D_FOUND - True if Open3D is found
# OPEN3D_INCLUDE_DIRS - Include directories for Open3D
# OPEN3D_LIBRARIES - Libraries to link against
# OPEN3D_VERSION - Version of Open3D found

find_package(PkgConfig QUIET)

# Try to find Open3D using pkg-config first
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_OPEN3D QUIET open3d)
endif()

# Find include directories
find_path(OPEN3D_INCLUDE_DIR
    NAMES open3d/Open3D.h
    HINTS
        ${PC_OPEN3D_INCLUDEDIR}
        ${PC_OPEN3D_INCLUDE_DIRS}
        $ENV{OPEN3D_ROOT}/include
        ${OPEN3D_ROOT}/include
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
        /opt/homebrew/include
        $ENV{CONDA_PREFIX}/include
        ${CONDA_PREFIX}/include
)

# Find library
find_library(OPEN3D_LIBRARY
    NAMES Open3D
    HINTS
        ${PC_OPEN3D_LIBDIR}
        ${PC_OPEN3D_LIBRARY_DIRS}
        $ENV{OPEN3D_ROOT}/lib
        ${OPEN3D_ROOT}/lib
    PATHS
        /usr/lib
        /usr/local/lib
        /opt/local/lib
        /opt/homebrew/lib
        $ENV{CONDA_PREFIX}/lib
        ${CONDA_PREFIX}/lib
)

# Extract version information if possible
if(PC_OPEN3D_VERSION)
    set(OPEN3D_VERSION ${PC_OPEN3D_VERSION})
elseif(OPEN3D_INCLUDE_DIR AND EXISTS "${OPEN3D_INCLUDE_DIR}/open3d/version.txt")
    file(STRINGS "${OPEN3D_INCLUDE_DIR}/open3d/version.txt" OPEN3D_VERSION LIMIT_COUNT 1)
endif()

# Handle the QUIETLY and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Open3D
    REQUIRED_VARS OPEN3D_LIBRARY OPEN3D_INCLUDE_DIR
    VERSION_VAR OPEN3D_VERSION
)

if(OPEN3D_FOUND)
    set(OPEN3D_LIBRARIES ${OPEN3D_LIBRARY})
    set(OPEN3D_INCLUDE_DIRS ${OPEN3D_INCLUDE_DIR})
    
    # Add additional dependencies if needed
    if(UNIX AND NOT APPLE)
        list(APPEND OPEN3D_LIBRARIES pthread)
    endif()
    
    # Create imported target
    if(NOT TARGET Open3D::Open3D)
        add_library(Open3D::Open3D UNKNOWN IMPORTED)
        set_target_properties(Open3D::Open3D PROPERTIES
            IMPORTED_LOCATION "${OPEN3D_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${OPEN3D_INCLUDE_DIR}"
        )
    endif()
endif()

# Mark variables as advanced
mark_as_advanced(OPEN3D_INCLUDE_DIR OPEN3D_LIBRARY)