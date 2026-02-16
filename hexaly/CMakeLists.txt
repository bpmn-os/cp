# Hexaly Solver Adapter Library

# Create the Hexaly adapter library
add_library(cp-hexaly
    hexaly_adapter.cpp
)

# Link dependencies
target_link_libraries(cp-hexaly
    PUBLIC
        cp                # Header-only cp.h interface
        ${HEXALY_DIR}/bin/libhexaly145.so
)

# Include directories
target_include_directories(cp-hexaly
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${HEXALY_DIR}/include
)

# Set RPATH for runtime library discovery
set_target_properties(cp-hexaly PROPERTIES
    BUILD_RPATH "${HEXALY_DIR}/bin"
    INSTALL_RPATH "${HEXALY_DIR}/bin"
)

# Set compiler options
target_compile_features(cp-hexaly PUBLIC cxx_std_23)
target_compile_options(cp-hexaly PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# Create alias for consistent naming
add_library(cp::hexaly ALIAS cp-hexaly)

# Install library
install(TARGETS cp-hexaly
    EXPORT cp-hexaly-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(FILES
    hexaly_adapter.h
    DESTINATION include/cp/hexaly
)

# Export targets
install(EXPORT cp-hexaly-targets
    FILE cp-hexaly-config.cmake
    NAMESPACE cp::
    DESTINATION lib/cmake/cp-hexaly
)
