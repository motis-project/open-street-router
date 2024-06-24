cmake_minimum_required(VERSION 3.10)
project(osr)

cmake_policy(SET CMP0079 NEW)

if (MSVC)
    # PDB debug information is not supported by buildcache.
    # Store debug info in the object files.
    option(OSR_DEBUG_SYMBOLS "generate debug symbols (debug builds)" ON)
    if (OSR_DEBUG_SYMBOLS)
        set(OSR_MSVC_DEBUG_FLAGS "/Z7")
    else ()
        set(OSR_MSVC_DEBUG_FLAGS "")
    endif ()
    string(REPLACE "/Zi" "${OSR_MSVC_DEBUG_FLAGS}" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
    string(REPLACE "/Zi" "${OSR_MSVC_DEBUG_FLAGS}" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/Zi" "${OSR_MSVC_DEBUG_FLAGS}" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
    string(REPLACE "/Zi" "${OSR_MSVC_DEBUG_FLAGS}" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

    add_definitions(-D_CRT_SECURE_NO_WARNINGS=1)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996")  # ignore deprecated fileno in tiles
endif ()

if (NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
    if (OSR_MIMALLOC)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    else ()
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif ()
endif ()

option(OSR_MIMALLOC "use mimalloc" OFF)
if (OSR_MIMALLOC AND WIN32)
    set(MI_BUILD_SHARED ON)
endif ()

include(cmake/buildcache.cmake)
include(cmake/pkg.cmake)

if (OSR_MIMALLOC)
    if (WIN32)
        set(osr-mimalloc-lib mimalloc)
        target_link_libraries(cista INTERFACE mimalloc)
    else ()
        set(osr-mimalloc-lib mimalloc-obj)
        target_link_libraries(cista INTERFACE mimalloc-static)
    endif ()
    target_compile_definitions(cista INTERFACE CISTA_USE_MIMALLOC=1)
    target_compile_definitions(boost INTERFACE BOOST_ASIO_DISABLE_STD_ALIGNED_ALLOC=1)
endif ()

# --- LINT ---
option(OSR_LINT "Run clang-tidy with the compiler." OFF)
if (OSR_LINT)
    # clang-tidy will be run on all targets defined hereafter
    include(cmake/clang-tidy.cmake)
endif ()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(osr-compile-options
            -Weverything
            -Wno-c++98-compat
            -Wno-c++98-compat-pedantic
            -Wno-newline-eof
            -Wno-missing-prototypes
            -Wno-padded
            -Wno-double-promotion
            -Wno-undef
            -Wno-undefined-reinterpret-cast
            -Wno-float-conversion
            -Wno-global-constructors
            -Wno-exit-time-destructors
            -Wno-switch-enum
            -Wno-c99-designator
            -Wno-zero-as-null-pointer-constant
            -Wno-missing-noreturn
            -Wno-undefined-func-template
            -Wno-unsafe-buffer-usage
            -Wno-c++20-compat
            -Wno-reserved-macro-identifier
            -Wno-documentation-unknown-command
            -Wno-duplicate-enum
            -Wno-ctad-maybe-unsupported
            -Wno-unknown-pragmas
            -Werror)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
    set(osr-compile-options -Wall -Wextra -Werror -Wno-unknown-pragmas)
elseif (MSVC)
    set(osr-compile-options /WX)
else ()
    set(osr-compile-options
            -Wall
            -Wextra
            -Wno-maybe-uninitialized)
    if (NOT CMAKE_CROSSCOMPILING)
        set(osr-compile-options ${osr-compile-options} -Werror)
    endif ()
endif ()

# --- LIB ---
file(GLOB_RECURSE osr-src src/*.cc)
add_library(osr ${osr-src})
target_include_directories(osr PUBLIC include)
target_compile_features(osr PUBLIC cxx_std_23)
target_compile_options(osr PRIVATE ${osr-compile-options})
target_link_libraries(osr
        osmium
        zlibstatic
        protozero
        expat
        geo
        cista
        utl
        tiles-import-library
        rtree
        unordered_dense
        boost-thread
)

# --- MAIN ---
add_executable(osr-extract exe/extract.cc)
target_link_libraries(osr-extract osr)

add_executable(osr-benchmark exe/benchmark.cc)
target_link_libraries(osr-benchmark osr)

file(GLOB_RECURSE osr-backend-src exe/backend/*.cc)
add_executable(osr-backend ${osr-backend-src})
target_link_libraries(osr-backend osr web-server conf boost-json)
target_include_directories(osr-backend PRIVATE exe/backend/include)

# --- TEST ---
configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dir.h.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/test_dir.h
)
file(GLOB_RECURSE osr-test-files test/*cc)
add_executable(osr-test ${osr-test-files})
target_include_directories(osr-test PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(osr-test gtest osr)

# --- MIMALLOC ---
if (OSR_MIMALLOC)
    target_link_libraries(osr-extract ${osr-mimalloc-lib})
    if (WIN32)
        add_custom_command(
                TARGET osr-extract POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy
                $<TARGET_FILE:mimalloc>
                $<TARGET_FILE_DIR:osr>
                COMMENT "Copy mimalloc.dll to output directory"
        )
        add_custom_command(
                TARGET osr-extract POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy
                "${CMAKE_CURRENT_BINARY_DIR}/deps/mimalloc/mimalloc-redirect.dll"
                $<TARGET_FILE_DIR:osr>
                COMMENT "Copy mimalloc-redirect.dll to output directory"
        )
    endif ()
endif ()