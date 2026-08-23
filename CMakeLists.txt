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

project(oscilloplot VERSION 1.0.0 LANGUAGES CXX)

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

find_package(SDL2 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(portaudio CONFIG REQUIRED)
find_package(SndFile CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

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
# Standalone Image Vectorizer Tool
#===============================================================================

add_executable(vectorizer_tool
    src/tools/vectorizer_tool.cpp
    src/vectorizer/image_vectorizer.cpp
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
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

target_include_directories(vectorizer_tool PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
    ${CMAKE_SOURCE_DIR}/external/stb
    ${CMAKE_SOURCE_DIR}/external/pfd
)

#===============================================================================
# Link Libraries
#
# The static vcpkg triplets export different target names than the dynamic
# ones (portaudio_static, SDL2::SDL2-static), so resolve them here.
#===============================================================================

if(TARGET portaudio)
    set(PORTAUDIO_TARGET portaudio)
else()
    set(PORTAUDIO_TARGET portaudio_static)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    OpenGL::GL
    ${PORTAUDIO_TARGET}
    SndFile::sndfile
    glm::glm
)

target_link_libraries(vectorizer_tool PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    OpenGL::GL
)

#===============================================================================
# Compiler Flags - Platform Specific
#===============================================================================

if(MSVC)
    # MSVC flags
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W4                     # Warning level 4
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

    # Vectorizer tool compile options
    target_compile_options(vectorizer_tool PRIVATE
        /W4 /MP /fp:fast /GS-
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Release>:/Ob3>
        $<$<CONFIG:Release>:/GL>
    )
    if(OSCILLOPLOT_ENABLE_SIMD AND NOT OSCILLOPLOT_SIMD_LEVEL STREQUAL "SSE2")
        target_compile_options(vectorizer_tool PRIVATE /arch:${OSCILLOPLOT_SIMD_LEVEL})
    endif()
    if(OSCILLOPLOT_ENABLE_LTO)
        target_link_options(vectorizer_tool PRIVATE $<$<CONFIG:Release>:/LTCG>)
    endif()
    target_compile_definitions(vectorizer_tool PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )

else()
    # GCC/Clang flags
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -Wextra -Wpedantic
        -ffast-math             # Fast floating-point
        -fno-exceptions         # Disable exceptions (embedded-style)
        -fno-rtti               # Disable RTTI (embedded-style)
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:Release>:-flto>
    )

    # SIMD flags for GCC/Clang - capped at the requested baseline so the binary
    # stays runnable on the oldest CPU the level implies, not just the build host.
    if(OSCILLOPLOT_ENABLE_SIMD)
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

    if(OSCILLOPLOT_STANDALONE)
        # GUI subsystem: launching from Explorer must not flash a console
        # window. SDL2main supplies WinMain and forwards to main().
        set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)
        set_target_properties(vectorizer_tool PROPERTIES WIN32_EXECUTABLE TRUE)
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
