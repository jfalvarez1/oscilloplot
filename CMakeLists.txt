cmake_minimum_required(VERSION 3.20)
project(oscilloplot VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Static linking of MSVC runtime - no VC++ Redistributable needed
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

#===============================================================================
# Build Options
#===============================================================================

option(OSCILLOPLOT_ENABLE_SIMD "Enable SIMD optimizations" ON)
option(OSCILLOPLOT_ENABLE_LTO "Enable Link-Time Optimization" ON)
option(OSCILLOPLOT_ENABLE_NATIVE "Enable native CPU optimizations" OFF)

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
)

#===============================================================================
# Link Libraries
#===============================================================================

target_link_libraries(${PROJECT_NAME} PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>
    OpenGL::GL
    portaudio
    SndFile::sndfile
    glm::glm
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

    # SIMD flags for MSVC
    if(OSCILLOPLOT_ENABLE_SIMD)
        target_compile_options(${PROJECT_NAME} PRIVATE
            /arch:AVX2          # Enable AVX2 instructions
        )
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
    target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -Wextra -Wpedantic
        -ffast-math             # Fast floating-point
        -fno-exceptions         # Disable exceptions (embedded-style)
        -fno-rtti               # Disable RTTI (embedded-style)
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:Release>:-flto>
    )

    # SIMD flags for GCC/Clang
    if(OSCILLOPLOT_ENABLE_SIMD)
        include(CheckCXXCompilerFlag)

        check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
        check_cxx_compiler_flag("-msse4.1" COMPILER_SUPPORTS_SSE41)
        check_cxx_compiler_flag("-msse2" COMPILER_SUPPORTS_SSE2)

        if(COMPILER_SUPPORTS_AVX2)
            target_compile_options(${PROJECT_NAME} PRIVATE -mavx2 -mfma)
            message(STATUS "Enabling AVX2 + FMA SIMD optimizations")
        elseif(COMPILER_SUPPORTS_SSE41)
            target_compile_options(${PROJECT_NAME} PRIVATE -msse4.1)
            message(STATUS "Enabling SSE4.1 SIMD optimizations")
        elseif(COMPILER_SUPPORTS_SSE2)
            target_compile_options(${PROJECT_NAME} PRIVATE -msse2)
            message(STATUS "Enabling SSE2 SIMD optimizations")
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
    # Copy SDL2.dll to output directory
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:SDL2::SDL2>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
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
message(STATUS "SIMD Enabled:   ${OSCILLOPLOT_ENABLE_SIMD}")
message(STATUS "LTO Enabled:    ${OSCILLOPLOT_ENABLE_LTO}")
message(STATUS "Native CPU:     ${OSCILLOPLOT_ENABLE_NATIVE}")
message(STATUS "C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "========================================")
message(STATUS "")
