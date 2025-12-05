# Oscilloplot

A high-performance, cross-platform oscilloscope XY audio generator written in C++.

## Features

- **Real-time XY oscilloscope visualization** at 120+ FPS
- **Physically-accurate CRT phosphor simulation** (Tektronix 465B inspired)
- **Low-latency audio playback** (<20ms latency)
- **Multiple pattern generators**:
  - Sine waves, circles, Lissajous curves
  - Hearts, stars, spirals
  - Sum of harmonics with frequency/phase sweep
  - Random harmonics
  - Freehand drawing canvas
  - Sound pad grid
- **3D Shape Generator** with 19 shapes:
  - Platonic solids: Cube, Tetrahedron, Octahedron, Icosahedron, Dodecahedron
  - Curved surfaces: Sphere, Torus, Cylinder, Cone
  - Prisms: Pyramid, N-sided Prism
  - Complex shapes: 3D Spiral, Double Helix, Trefoil Knot
  - Topological: Mobius Strip, Klein Bottle
  - Special: Spring, 3D Star
  - **3D Text**: Type any text and render it as a rotating 3D wireframe!
- **Rich effects pipeline**:
  - Rotation (static/animated)
  - X/Y axis fading
  - Shrink/unshrink
  - Noise injection
  - Wavy modulation
  - Tremolo
  - Ring modulation
  - Echo/delay
  - Kaleidoscope
- **File I/O**:
  - Load/save patterns (text, binary)
  - Export to WAV audio files
- **Cross-platform**: Windows, macOS, Linux

## Technology Stack

| Component | Library |
|-----------|---------|
| Windowing/Input | SDL2 |
| GUI | Dear ImGui |
| Plotting | ImPlot |
| Audio I/O | PortAudio |
| Audio Files | libsndfile |
| FFT | PocketFFT |
| Math | GLM |

## Building

### Prerequisites

- CMake 3.20+
- C++17 compiler (MSVC 2019+, GCC 9+, Clang 10+)
- Git

### Windows (with vcpkg)

```powershell
# 1. Setup external dependencies (ImGui, ImPlot, PocketFFT)
./setup_externals.ps1

# 2. Install vcpkg dependencies
vcpkg install sdl2 portaudio libsndfile glm --triplet=x64-windows

# 3. Configure and build
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE="[vcpkg-root]/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release

# 4. Run
./build/Release/oscilloplot.exe
```

### macOS

```bash
# 1. Setup external dependencies
chmod +x setup_externals.sh
./setup_externals.sh

# 2. Install dependencies via Homebrew
brew install sdl2 portaudio libsndfile glm

# 3. Configure and build
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build

# 4. Run
./build/oscilloplot
```

### Linux (Ubuntu/Debian)

```bash
# 1. Setup external dependencies
chmod +x setup_externals.sh
./setup_externals.sh

# 2. Install dependencies
sudo apt update
sudo apt install libsdl2-dev libportaudio2 libportaudiocpp0 portaudio19-dev \
                 libsndfile1-dev libglm-dev libgl1-mesa-dev

# 3. Configure and build
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build

# 4. Run
./build/oscilloplot
```

## Usage

### Keyboard Shortcuts

- **Space**: Play/Stop
- **Escape**: Exit

### Quick Start

1. Launch the application
2. Select a pattern generator from the Generators panel
3. Adjust audio parameters (sample rate, duration, repeats)
4. Enable effects as desired
5. Press Space or click Play to hear the audio and see the visualization

### Audio Output

The application outputs stereo audio where:
- **Left channel (X)**: Horizontal deflection
- **Right channel (Y)**: Vertical deflection

Connect to an oscilloscope in X-Y mode to visualize the patterns on hardware.

## Performance Targets

| Metric | Target |
|--------|--------|
| Preview FPS | 120+ |
| CPU Usage | <20% |
| Audio Latency | <20ms |
| Memory Usage | <50MB |

## Future Roadmap

- [ ] MIDI input support
- [ ] VST plugin hosting
- [ ] Pattern sequencing/timeline
- [ ] More 3D shape import (OBJ files)

## License

MIT License - see [LICENSE](LICENSE) for details.

## Acknowledgments

- [Dear ImGui](https://github.com/ocornut/imgui) - Immediate mode GUI
- [ImPlot](https://github.com/epezent/implot) - Real-time plotting
- [PortAudio](http://www.portaudio.com/) - Cross-platform audio
- Original Python version: [oscilloplot_gui](https://github.com/jfalvarez1/oscilloplot_gui)
