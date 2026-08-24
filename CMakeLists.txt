cmake_minimum_required(VERSION 3.20)

#===============================================================================
# Standalone build selection
#
# This must run BEFORE project(), because that is where the vcpkg toolchain file
# gets included and the triplet is locked in. A standalone build links every
# dependency statically, so the release is a single .exe with no SDL2.dll /
# portaudio.dll / sndfile.dll beside it.
#===============================================================================

option(OSCILLOPLOT_STANDALONE "Build a single self-contained executable (no DLLs, no console)" ON)

if(OSCILLOPLOT_STANDALONE AND CMAKE_HOST_WIN32 AND NOT DEFINED VCPKG_TARGET_TRIPLET)
    if(CMAKE_GENERATOR_PLATFORM STREQUAL "Win32")
        set(VCPKG_TARGET_TRIPLET "x86-windows-static" CACHE STRING "")
    elseif(CMAKE_GENERATOR_PLATFORM STREQUAL "ARM64")
        set(VCPKG_TARGET_TRIPLET "arm64-windows-static" CACHE STRING "")
    else()
        set(VCPKG_TARGET_TRIPLET "x64-windows-static" CACHE STRING "")
    endif()
endif()

project(oscilloplot VERSION 1.1.0 LANGUAGES CXX)

# Single source of truth for the version string shown in the title bar,
# the About box, and the Windows version resource.
add_compile_definitions(OSCILLOPLOT_VERSION="${PROJECT_VERSION}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CRT linkage must match the vcpkg triplet, or the link mixes LIBCMT with
# MSVCRT (LNK4098) and the two heaps disagree at runtime.
#   standalone  -> *-windows-static triplet -> static CRT, no VC++ Redistributable
#   dynamic     -> *-windows triplet        -> DLL CRT, matching the shipped DLLs
if(OSCILLOPLOT_STANDALONE)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

#===============================================================================
# Build Options
#===============================================================================

option(OSCILLOPLOT_ENABLE_SIMD "Enable SIMD optimizations" ON)
option(OSCILLOPLOT_ENABLE_LTO "Enable Link-Time Optimization" ON)
option(OSCILLOPLOT_ENABLE_NATIVE "Enable native CPU optimizations" OFF)
# SIMD baseline. The default (SSE2) is the x86-64 baseline every CPU capable of
# running Windows 11 supports, including machines running unsupported/bypass
# installs on pre-AVX hardware. AVX2 is measurably faster but faults with
# "illegal instruction" on anything older than Haswell (2013) / Excavator (2015),
# so it stays opt-in for self-built binaries rather than shipped in releases.
set(OSCILLOPLOT_SIMD_LEVEL "SSE2" CACHE STRING "SIMD baseline: SSE2, AVX, or AVX2")
set_property(CACHE OSCILLOPLOT_SIMD_LEVEL PROPERTY STRINGS SSE2 AVX AVX2)

#===============================================================================
# vcpkg integration
#===============================================================================

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "")
endif()

#===============================================================================
# Find Dependencies
#===============================================================================

# vcpkg ships CMake config packages for everything; Homebrew and most Linux
# distros ship pkg-config for portaudio (and sometimes SDL2/sndfile) instead.
# Try config mode first, then fall back to pkg-config, so one CMakeLists works
# on Windows/vcpkg, macOS/Homebrew and Linux alike.
find_package(OpenGL REQUIRED)
find_package(PkgConfig QUIET)

# --- SDL2 -------------------------------------------------------------------
find_package(SDL2 CONFIG QUIET)
if(TARGET SDL2::SDL2)
    set(SDL2_LINK_TARGET SDL2::SDL2)
elseif(TARGET SDL2::SDL2-static)
    set(SDL2_LINK_TARGET SDL2::SDL2-static)
else()
    if(NOT PkgConfig_FOUND)
        message(FATAL_ERROR "SDL2 not found and pkg-config unavailable")
    endif()
    pkg_check_modules(SDL2_PC REQUIRED IMPORTED_TARGET sdl2)
    set(SDL2_LINK_TARGET PkgConfig::SDL2_PC)
endif()
message(STATUS "SDL2 target:      ${SDL2_LINK_TARGET}")

# --- PortAudio --------------------------------------------------------------
find_package(portaudio CONFIG QUIET)
if(TARGET portaudio)
    set(PORTAUDIO_LINK_TARGET portaudio)
elseif(TARGET portaudio_static)
    set(PORTAUDIO_LINK_TARGET portaudio_static)
else()
    if(NOT PkgConfig_FOUND)
        message(FATAL_ERROR "PortAudio not found and pkg-config unavailable")
    endif()
    pkg_check_modules(PORTAUDIO_PC REQUIRED IMPORTED_TARGET portaudio-2.0)
    set(PORTAUDIO_LINK_TARGET PkgConfig::PORTAUDIO_PC)
endif()
message(STATUS "PortAudio target: ${PORTAUDIO_LINK_TARGET}")

# --- libsndfile -------------------------------------------------------------
find_package(SndFile CONFIG QUIET)
if(TARGET SndFile::sndfile)
    set(SNDFILE_LINK_TARGET SndFile::sndfile)
else()
    if(NOT PkgConfig_FOUND)
        message(FATAL_ERROR "libsndfile not found and pkg-config unavailable")
    endif()
    pkg_check_modules(SNDFILE_PC REQUIRED IMPORTED_TARGET sndfile)
    set(SNDFILE_LINK_TARGET PkgConfig::SNDFILE_PC)
endif()
message(STATUS "libsndfile target: ${SNDFILE_LINK_TARGET}")

# glm was a declared dependency but no source file includes it, so it is not
# searched for here.

#===============================================================================
# ImGui and ImPlot (vendored)
#===============================================================================

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
set(IMPLOT_DIR ${CMAKE_SOURCE_DIR}/external/implot)

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

set(IMPLOT_SOURCES
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
)

# Vendored code compiles at /W0 (MSVC) / -w (GCC/Clang) so that project
# warnings are the only warnings the build prints.
if(MSVC)
    set_source_files_properties(${IMGUI_SOURCES} ${IMPLOT_SOURCES} PROPERTIES COMPILE_OPTIONS "/W0")
else()
    set_source_files_properties(${IMGUI_SOURCES} ${IMPLOT_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
endif()

#===============================================================================
# Application Sources
#===============================================================================

set(APP_SOURCES
    src/main.cpp
    src/app.cpp
    src/audio/audio_engine.cpp
    src/audio/audio_buffer.cpp
    src/audio/wav_export.cpp
    src/audio/effects/effect_chain.cpp
    src/audio/effects/rotation.cpp
    src/audio/effects/fade.cpp
    src/audio/effects/noise.cpp
    src/audio/effects/wavy.cpp
    src/audio/effects/tremolo.cpp
    src/audio/effects/ring_mod.cpp
    src/audio/effects/echo.cpp
    src/audio/effects/kaleidoscope.cpp
    src/generators/test_pattern.cpp
    src/generators/harmonics.cpp
    src/generators/spiral.cpp
    src/generators/random_harmonics.cpp
    src/ui/ui_manager.cpp
    src/ui/main_window.cpp
    src/ui/control_panel.cpp
    src/ui/drawing_canvas.cpp
    src/ui/sound_pad.cpp
    src/data/pattern.cpp
    src/data/file_loader.cpp
    src/data/file_saver.cpp
    src/data/obj_loader.cpp
    src/vectorizer/image_vectorizer.cpp
)

# Header-only optimized utilities (for IDE indexing)
set(UTILITY_HEADERS
    src/utils/dsp_core.hpp
    src/utils/lock_free_queue.hpp
    src/utils/memory_pool.hpp
    src/audio/effects/effect_processor.hpp
    src/audio/realtime_audio.hpp
)

#===============================================================================
# Create Executable
#===============================================================================

add_executable(${PROJECT_NAME}
    ${APP_SOURCES}
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
    ${UTILITY_HEADERS}
)

#===============================================================================
# Include Directories
#===============================================================================

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
    ${CMAKE_SOURCE_DIR}/external/pocketfft
    ${CMAKE_SOURCE_DIR}/external/stb
    ${CMAKE_SOURCE_DIR}/external/pfd
)

#===============================================================================
# Link Libraries
#
# The static vcpkg triplets export different target names than the dynamic
# ones (portaudio_static, SDL2::SDL2-static), so resolve them here.
#===============================================================================


target_link_libraries(${PROJECT_NAME} PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    ${SDL2_LINK_TARGET}
    OpenGL::GL
    ${PORTAUDIO_LINK_TARGET}
    ${SNDFILE_LINK_TARGET}
)

#===============================================================================
# Compiler Flags - Platform Specific
#===============================================================================

if(MSVC)
    # MSVC flags
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W4                     # Warning level 4
        /wd4324                 # "padded due to alignment specifier" - alignas is deliberate
        /MP                     # Multi-processor compilation
        /fp:fast                # Fast floating-point
        /GS-                    # Disable buffer security checks (perf)
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Release>:/Ob3>    # Maximum inlining
        $<$<CONFIG:Release>:/GL>     # Whole program optimization
    )

    # SIMD flags for MSVC. SSE2 is implicit on x64 and MSVC rejects /arch:SSE2
    # there, so only AVX/AVX2 add a flag.
    if(OSCILLOPLOT_ENABLE_SIMD AND NOT OSCILLOPLOT_SIMD_LEVEL STREQUAL "SSE2")
        target_compile_options(${PROJECT_NAME} PRIVATE /arch:${OSCILLOPLOT_SIMD_LEVEL})
    endif()

    # Link-time optimization
    if(OSCILLOPLOT_ENABLE_LTO)
        target_link_options(${PROJECT_NAME} PRIVATE
            $<$<CONFIG:Release>:/LTCG>
        )
    endif()

    target_compile_definitions(${PROJECT_NAME} PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )

else()
    # GCC/Clang flags
    # NOTE: -fno-exceptions was here but the code genuinely uses try/catch
    # (main.cpp's top-level handler, file_loader's std::stof guard), so the
    # GCC/Clang build could not compile at all. MSVC has always built with
    # exceptions enabled; the flags now agree.
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -Wextra -Wpedantic
        -ffast-math             # Fast floating-point
        -fno-rtti               # Disable RTTI (nothing uses dynamic_cast/typeid)
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:Release>:-flto>
    )

    # SIMD flags for GCC/Clang - capped at the requested baseline so the binary
    # stays runnable on the oldest CPU the level implies, not just the build host.
    # On ARM (Apple Silicon, aarch64) NEON is part of the baseline ABI and the
    # x86 switches do not exist, so there is nothing to add.
    if(OSCILLOPLOT_ENABLE_SIMD AND NOT CMAKE_SYSTEM_PROCESSOR MATCHES "arm|aarch64|ARM64")
        include(CheckCXXCompilerFlag)

        if(OSCILLOPLOT_SIMD_LEVEL STREQUAL "AVX2")
            check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
            if(COMPILER_SUPPORTS_AVX2)
                target_compile_options(${PROJECT_NAME} PRIVATE -mavx2 -mfma)
                message(STATUS "Enabling AVX2 + FMA SIMD optimizations")
            endif()
        elseif(OSCILLOPLOT_SIMD_LEVEL STREQUAL "AVX")
            check_cxx_compiler_flag("-mavx" COMPILER_SUPPORTS_AVX)
            if(COMPILER_SUPPORTS_AVX)
                target_compile_options(${PROJECT_NAME} PRIVATE -mavx)
                message(STATUS "Enabling AVX SIMD optimizations")
            endif()
        else()
            check_cxx_compiler_flag("-msse2" COMPILER_SUPPORTS_SSE2)
            if(COMPILER_SUPPORTS_SSE2)
                target_compile_options(${PROJECT_NAME} PRIVATE -msse2)
                message(STATUS "Enabling SSE2 SIMD optimizations (portable baseline)")
            endif()
        endif()
    endif()

    # Native CPU optimizations (optional, reduces portability)
    if(OSCILLOPLOT_ENABLE_NATIVE)
        target_compile_options(${PROJECT_NAME} PRIVATE -march=native)
        message(STATUS "Enabling native CPU optimizations")
    endif()

    # Link-time optimization
    if(OSCILLOPLOT_ENABLE_LTO)
        target_link_options(${PROJECT_NAME} PRIVATE
            $<$<CONFIG:Release>:-flto>
        )
    endif()
endif()

#===============================================================================
# Platform-Specific Configuration
#===============================================================================

if(WIN32)
    # Application manifest: declares Windows 10/11 support (so version APIs and
    # shell behaviour aren't shimmed), per-monitor-v2 DPI awareness, long paths,
    # and a UTF-8 ANSI code page.
    target_sources(${PROJECT_NAME} PRIVATE assets/windows/oscilloplot.manifest)

    # Icon + version resource, generated so FILEVERSION tracks PROJECT_VERSION.
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/assets/windows/oscilloplot.ico" OSCILLOPLOT_ICO_NATIVE)
    string(REPLACE "\\" "\\\\" OSCILLOPLOT_ICO_PATH "${OSCILLOPLOT_ICO_NATIVE}")
    configure_file(assets/windows/oscilloplot.rc.in ${CMAKE_BINARY_DIR}/oscilloplot.rc @ONLY)
    target_sources(${PROJECT_NAME} PRIVATE ${CMAKE_BINARY_DIR}/oscilloplot.rc)

    if(OSCILLOPLOT_STANDALONE)
        # GUI subsystem: launching from Explorer must not flash a console
        # window. SDL2main supplies WinMain and forwards to main().
        set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
    else()
        # Dynamic build - the dependency DLLs must sit next to the executable.
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
    endif()
endif()

if(APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        "-framework CoreFoundation"
        "-framework CoreAudio"
        "-framework AudioToolbox"
        "-framework CoreServices"
        "-framework AppKit"
    )

    # Build a real .app bundle so the icon, name and Retina flag are picked up
    # by Finder and the Dock, and the app does not launch as a terminal tool.
    set(MACOSX_ICON ${CMAKE_SOURCE_DIR}/assets/macos/oscilloplot.icns)
    set_source_files_properties(${MACOSX_ICON} PROPERTIES
        MACOSX_PACKAGE_LOCATION Resources)
    target_sources(${PROJECT_NAME} PRIVATE ${MACOSX_ICON})

    configure_file(assets/macos/Info.plist.in ${CMAKE_BINARY_DIR}/Info.plist @ONLY)

    set_target_properties(${PROJECT_NAME} PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_BINARY_DIR}/Info.plist
        OUTPUT_NAME "oscilloplot"
    )
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        pthread
        dl
    )
endif()

#===============================================================================
# Install Configuration
#===============================================================================

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME}/assets)

#===============================================================================
# Print Configuration Summary
#===============================================================================

message(STATUS "")
message(STATUS "=== Oscilloplot Build Configuration ===")
message(STATUS "Version:        ${PROJECT_VERSION}")
message(STATUS "Build Type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "SIMD Enabled:   ${OSCILLOPLOT_ENABLE_SIMD} (${OSCILLOPLOT_SIMD_LEVEL})")
message(STATUS "Standalone:     ${OSCILLOPLOT_STANDALONE}")
message(STATUS "vcpkg Triplet:  ${VCPKG_TARGET_TRIPLET}")
message(STATUS "LTO Enabled:    ${OSCILLOPLOT_ENABLE_LTO}")
message(STATUS "Native CPU:     ${OSCILLOPLOT_ENABLE_NATIVE}")
message(STATUS "C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "========================================")
message(STATUS "")
