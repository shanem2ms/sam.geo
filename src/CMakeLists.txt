if (${CMAKE_CXX_FLAGS_RELEASE})
STRING(REPLACE "O2" "Od" CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS_RELEASE})
endif()

################################################################################
# Source groups
################################################################################
set(Header_Files
    "spherical_mercator_projection.h"
    "spherical_projection.h"
    "tile_selection.h"
    "tile3d_selection.h"
    "tileloc.h"
)  

source_group("Header Files" FILES ${Header_Files})

set(Source_Files
    "spherical_mercator_projection.cpp"
    "spherical_projection.cpp"
    "tile_selection.cpp"
    "tile3d_selection.cpp"
    "tileloc.cpp"
    )
    
source_group("Source Files" FILES ${Source_Files} ${Main_Files})

set(ALL_FILES
    
    ${Header_Files} 
    ${Source_Files}
)


add_library(sam.geo STATIC ${ALL_FILES})

################################################################################
# Include directories
################################################################################
target_include_directories(sam.geo PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/."
    "${VCPKG_INCLUDEDIR}"
    "${EXT_INCLUDES}"
    )

add_compile_definitions(DLLX=;PRId64="I64d")
