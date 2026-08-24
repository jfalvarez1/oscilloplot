# Oscilloplot Test Plan

Two layers: an automated suite for logic that can run headless, and a manual
checklist for anything needing a window, a GPU or an audio device.

## Automated suite

Built as `oscilloplot_tests` and run by CTest on every CI build.

```bash
cmake --build build --config Release --target oscilloplot_tests
ctest --test-dir build -C Release --output-on-failure
# or run directly, optionally filtered:
./build/Release/oscilloplot_tests ObjLoader
```

Covered:

| Area | What is asserted |
|------|------------------|
| `Pattern` | push/clear/size, normalise centring and range, empty and single-point safety |
| `FileIO` | binary round-trip is exact; text round-trip preserves shape; comments and both separators; MATLAB arrays; missing/empty files rejected |
| `ObjLoader` | cube edge de-duplication, centring and scaling, polylines, negative indices, point clouds and empty files rejected, degenerate faces, decimation cap |
| `Sequence` | `.oseq` round-trip, v1 backward compatibility, CRLF names, bad magic, truncated files, absurd step counts, empty-save refusal |
| `Crossfade` | endpoints return originals, midpoint of concentric circles has the mean radius, mismatched sizes, empty inputs |
| `RecentFiles` | ordering, de-duplication, cap, disk round-trip, pruning missing files, display names |
| `UndoStack` | undo/redo traversal, empty behaviour, redo branch invalidation, bounded depth |
| `Preset` | key lookup and replacement, round-trip, forward compatibility with unknown keys, bad magic |
| `SineLUT` | accuracy against `std::sin`, phase wrapping, range, cos/sin quarter-turn relationship |
| `Generators` | every generator produces the requested count, finite values, in-range points; spiral grows outward; tiny point counts do not crash |

Deliberately out of scope: anything requiring ImGui, an OpenGL context or a
PortAudio stream. Those are covered below.

## Manual checklist

Run against a release build before tagging.

### Startup and platform
- [ ] Launches with no console window (Windows) and shows the app icon
- [ ] Title bar reads `Oscilloplot <version> - XY Audio Generator`
- [ ] `Help > About` shows the version, GL renderer and build date
- [ ] Closing the window exits with code 0 (no crash dialog)
- [ ] Deleting `imgui.ini` and relaunching gives the default layout with the menu bar visible
- [ ] `View > Reset Layout` restores panel positions

### Audio
- [ ] Play produces sound; Stop silences it
- [ ] Space toggles playback, but not while typing in a text field
- [ ] With no output device the app still opens and shows `[No audio device]`

### Generators
- [ ] Clicking a generator draws it immediately
- [ ] Dragging that generator's sliders updates the scope live
- [ ] The shared Points slider re-runs the active generator
- [ ] Switching generators hands the live sliders to the new one

### Files
- [ ] Ctrl+O loads `.txt`, `.csv`, `.osc` and `.m` files
- [ ] Ctrl+S saves; typing `.osc` writes binary, no extension writes text
- [ ] Ctrl+E exports a WAV that opens in an audio editor, with the expected length
- [ ] Recent files list populates, reopens, and drops deleted files
- [ ] Loading a pattern stops the 3D animation from overwriting it

### 3D shapes
- [ ] Each shape in the dropdown renders
- [ ] Animation runs and rotation sliders update the shape live
- [ ] OBJ import loads a mesh, auto-centred and scaled; Max Edges thins it
- [ ] Bake Rotation produces a rotating pattern that survives WAV export

### Sequencer
- [ ] Capture adds the current pattern as a step
- [ ] Play advances through steps at roughly the configured cycle counts
- [ ] Crossfade blends between steps rather than hard-switching
- [ ] Reorder and delete keep the playing step tracked
- [ ] Save/Load `.oseq` round-trips
- [ ] Export WAV renders the whole sequence

### Sound Pad and Canvas
- [ ] Clicking a pad cell toggles it and updates the scope immediately
- [ ] Dragging a cell moves its point
- [ ] Canvas strokes draw; Undo removes the last stroke; Clear asks first

### Vectorizer
- [ ] Loading an image and processing produces a traceable outline
- [ ] Apply to Oscilloscope shows it on the scope
- [ ] Detail Emphasis visibly changes the amount of retained detail

### Display
- [ ] Phosphor presets change the trace colour
- [ ] Effects are visible on the scope while stopped, not only during playback
- [ ] Export Image writes a PNG matching what is on screen
