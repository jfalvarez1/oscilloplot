#!/bin/bash
# Setup script for Oscilloplot C++ external dependencies
# Run this script from the project root directory

set -e

echo -e "\033[36mSetting up external dependencies for Oscilloplot...\033[0m"

# Create external directory if it doesn't exist
mkdir -p external
cd external

# Clone Dear ImGui
echo -e "\n\033[33mCloning Dear ImGui...\033[0m"
if [ ! -d "imgui" ]; then
    git clone --depth 1 --branch v1.91.6 https://github.com/ocornut/imgui.git
else
    echo -e "\033[32mImGui already exists, skipping...\033[0m"
fi

# Clone ImPlot
echo -e "\n\033[33mCloning ImPlot...\033[0m"
if [ ! -d "implot" ]; then
    git clone --depth 1 --branch v0.16 https://github.com/epezent/implot.git
else
    echo -e "\033[32mImPlot already exists, skipping...\033[0m"
fi

# Clone PocketFFT (header-only)
echo -e "\n\033[33mCloning PocketFFT...\033[0m"
if [ ! -d "pocketfft" ]; then
    git clone --depth 1 https://github.com/mreineck/pocketfft.git
else
    echo -e "\033[32mPocketFFT already exists, skipping...\033[0m"
fi

cd ..

echo -e "\n\033[36m========================================\033[0m"
echo -e "\033[32mExternal dependencies setup complete!\033[0m"
echo -e "\033[36m========================================\n\033[0m"

echo -e "\033[33mNext steps:\033[0m"
echo "1. Install dependencies via your package manager or vcpkg:"
echo ""
echo "   # macOS (Homebrew):"
echo "   brew install sdl2 portaudio libsndfile glm"
echo ""
echo "   # Ubuntu/Debian:"
echo "   sudo apt install libsdl2-dev libportaudio2 libsndfile1-dev libglm-dev"
echo ""
echo "   # Or use vcpkg:"
echo "   vcpkg install sdl2 portaudio libsndfile glm"
echo ""
echo "2. Configure and build:"
echo "   cmake -B build -S . -DCMAKE_BUILD_TYPE=Release"
echo "   cmake --build build"
echo ""
