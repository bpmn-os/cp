cmake_minimum_required(VERSION 3.15)
project(cp-solvers VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
endif()

# Options
option(BUILD_SCIP_ADAPTER "Build SCIP solver adapter" ON)
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build test suite" ON)

# Header-only interface library for cp.h
add_library(cp INTERFACE)
target_include_directories(cp INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# SCIP Adapter
if(BUILD_SCIP_ADAPTER)
    find_package(SCIP QUIET)
    if(NOT SCIP_FOUND)
        message(STATUS "SCIP not found - downloading and building from source...")
        include(FetchContent)
        set(AUTOBUILD ON CACHE BOOL "Auto-detect optional SCIP dependencies")
        set(PAPILO OFF CACHE BOOL "Disable papilo")
        set(BUILD_TESTING OFF CACHE BOOL "Disable SCIP tests")
        set(SCIP_BUILD_TESTING OFF CACHE BOOL "Disable SCIP tests")
        FetchContent_Declare(
            scip
            URL https://scipopt.org/download/release/scipoptsuite-10.0.1.tgz
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_MakeAvailable(scip)
        set(SCIP_DIR ${scip_BINARY_DIR})
    else()
        message(STATUS "SCIP found - using system installation")
    endif()
    add_subdirectory(scip)
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install header-only interface
install(FILES cp.h solver.h
    DESTINATION include/cp
)

# Export targets
install(TARGETS cp
    EXPORT cp-targets
    INCLUDES DESTINATION include
)

install(EXPORT cp-targets
    FILE cp-config.cmake
    NAMESPACE cp::
    DESTINATION lib/cmake/cp
)
