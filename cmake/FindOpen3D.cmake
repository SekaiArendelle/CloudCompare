# Find Open3D library
# This module finds the Open3D library and sets the following variables:
# Open3D_FOUND - True if Open3D is found
# Open3D_INCLUDE_DIRS - Include directories for Open3D
# Open3D_LIBRARIES - Libraries to link against
# Open3D_VERSION - Version of Open3D found

# For pixi/conda environments, Open3D is typically available through the environment
if(DEFINED ENV{CONDA_PREFIX})
    set(Open3D_ROOT_PATH $ENV{CONDA_PREFIX})
elseif(DEFINED ENV{PIXI_ENVIRONMENT_DIR})
    set(Open3D_ROOT_PATH $ENV{PIXI_ENVIRONMENT_DIR})
endif()

# Find include directories
find_path(Open3D_INCLUDE_DIR
    NAMES open3d/Open3D.h
    HINTS
        ${Open3D_ROOT_PATH}/include
        $ENV{OPEN3D_ROOT}/include
        ${OPEN3D_ROOT}/include
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
        /opt/homebrew/include
)

# Find library
find_library(Open3D_LIBRARY
    NAMES Open3D
    HINTS
        ${Open3D_ROOT_PATH}/lib
        $ENV{OPEN3D_ROOT}/lib
        ${OPEN3D_ROOT}/lib
    PATHS
        /usr/lib
        /usr/local/lib
        /opt/local/lib
        /opt/homebrew/lib
)

# Extract version information if possible
if(Open3D_INCLUDE_DIR AND EXISTS "${Open3D_INCLUDE_DIR}/open3d/version.txt")
    file(STRINGS "${Open3D_INCLUDE_DIR}/open3d/version.txt" Open3D_VERSION LIMIT_COUNT 1)
endif()

# Handle the QUIETLY and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Open3D
    REQUIRED_VARS Open3D_LIBRARY Open3D_INCLUDE_DIR
    VERSION_VAR Open3D_VERSION
)

if(Open3D_FOUND)
    set(Open3D_LIBRARIES ${Open3D_LIBRARY})
    set(Open3D_INCLUDE_DIRS ${Open3D_INCLUDE_DIR})
    
    # Add additional dependencies if needed
    if(UNIX AND NOT APPLE)
        list(APPEND Open3D_LIBRARIES pthread)
    endif()
    
    # Create imported target
    if(NOT TARGET Open3D::Open3D)
        add_library(Open3D::Open3D UNKNOWN IMPORTED)
        set_target_properties(Open3D::Open3D PROPERTIES
            IMPORTED_LOCATION "${Open3D_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${Open3D_INCLUDE_DIR}"
        )
    endif()
endif()

# Mark variables as advanced
mark_as_advanced(Open3D_INCLUDE_DIR Open3D_LIBRARY)