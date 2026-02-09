# SCIP Solver Adapter Library

# Find SCIP (either from system or FetchContent)
if(NOT TARGET libscip)
    find_package(SCIP REQUIRED)
endif()

# Create the SCIP adapter library
add_library(cp-scip
    scip_adapter.cpp
)

# Link dependencies
if(TARGET libscip)
    # SCIP from FetchContent
    target_link_libraries(cp-scip
        PUBLIC
            cp                # Header-only cp.h interface
            libscip
        PRIVATE
            ${CMAKE_DL_LIBS}
    )
else()
    # SCIP from system installation
    target_link_libraries(cp-scip
        PUBLIC
            cp                # Header-only cp.h interface
            ${SCIP_LIBRARIES}
        PRIVATE
            ${CMAKE_DL_LIBS}
    )
endif()

# Include directories
if(TARGET libscip)
    # SCIP from FetchContent - includes come from target
    target_include_directories(cp-scip
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>
    )
else()
    # SCIP from system installation
    target_include_directories(cp-scip
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>
            ${SCIP_INCLUDE_DIRS}
    )
endif()

# Set compiler options
target_compile_features(cp-scip PUBLIC cxx_std_23)
target_compile_options(cp-scip PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# Create alias for consistent naming
add_library(cp::scip ALIAS cp-scip)

# Install library
install(TARGETS cp-scip
    EXPORT cp-scip-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(FILES
    scip_adapter.h
    DESTINATION include/cp/scip
)

# Export targets
install(EXPORT cp-scip-targets
    FILE cp-scip-config.cmake
    NAMESPACE cp::
    DESTINATION lib/cmake/cp-scip
)
