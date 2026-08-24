//==============================================================================
// Tests for the pure data layer: patterns, file I/O, OBJ import, sequences,
// presets, recent files and the undo stack.
//==============================================================================

#include "test_framework.hpp"

#include "data/file_loader.hpp"
#include "data/file_saver.hpp"
#include "data/obj_loader.hpp"
#include "data/pattern.hpp"
#include "data/preset.hpp"
#include "data/recent_files.hpp"
#include "data/sequence.hpp"
#include "utils/undo_stack.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

using namespace oscilloplot;

namespace {

// Tests write scratch files into the working directory and clean up after.
struct TempFile {
    std::string path;
    explicit TempFile(const char* name) : path(name) {}
    ~TempFile() { std::remove(path.c_str()); }
    void write(const std::string& contents) const {
        std::ofstream f(path);
        f << contents;
    }
};

Pattern makeCircle(size_t n, float radius = 0.9f) {
    Pattern p;
    for (size_t i = 0; i < n; ++i) {
        float t = static_cast<float>(i) * 6.28318530718f / static_cast<float>(n);
        p.push_back(std::cos(t) * radius, std::sin(t) * radius);
    }
    return p;
}

} // namespace

//==============================================================================
// Pattern
//==============================================================================

TEST(Pattern, push_back_and_size) {
    Pattern p;
    CHECK(p.empty());
    p.push_back(0.5f, -0.25f);
    CHECK_EQ(p.size(), size_t(1));
    CHECK_NEAR(p.x[0], 0.5, 1e-6);
    CHECK_NEAR(p.y[0], -0.25, 1e-6);
    p.clear();
    CHECK(p.empty());
}

TEST(Pattern, normalize_fills_range) {
    Pattern p;
    p.push_back(10.0f, 20.0f);
    p.push_back(12.0f, 24.0f);
    p.normalize();
    // Largest dimension should span [-1, 1] and stay centred.
    float minX = p.x[0], maxX = p.x[0], minY = p.y[0], maxY = p.y[0];
    for (size_t i = 0; i < p.size(); ++i) {
        minX = std::fmin(minX, p.x[i]); maxX = std::fmax(maxX, p.x[i]);
        minY = std::fmin(minY, p.y[i]); maxY = std::fmax(maxY, p.y[i]);
    }
    CHECK_NEAR((minY + maxY) / 2.0, 0.0, 1e-5);
    CHECK_NEAR(maxY - minY, 2.0, 1e-5);      // Y had the larger range
    CHECK(maxX <= 1.0001f && minX >= -1.0001f);
}

TEST(Pattern, normalize_empty_is_safe) {
    Pattern p;
    p.normalize();          // must not crash or read out of bounds
    CHECK(p.empty());
}

TEST(Pattern, normalize_single_point_is_safe) {
    Pattern p;
    p.push_back(5.0f, 5.0f);
    p.normalize();          // zero range - must not divide by zero
    CHECK_EQ(p.size(), size_t(1));
    CHECK(std::isfinite(p.x[0]));
    CHECK(std::isfinite(p.y[0]));
}

//==============================================================================
// File I/O
//==============================================================================

TEST(FileIO, binary_round_trip_is_exact) {
    TempFile tf("test_rt.osc");
    Pattern src = makeCircle(500);

    CHECK(FileSaver::saveBinaryFile(tf.path, src));

    Pattern loaded;
    CHECK(FileLoader::loadBinaryFile(tf.path, loaded));
    CHECK_EQ(loaded.size(), src.size());

    double worst = 0.0;
    for (size_t i = 0; i < src.size() && i < loaded.size(); ++i) {
        worst = std::fmax(worst, std::fabs(src.x[i] - loaded.x[i]));
        worst = std::fmax(worst, std::fabs(src.y[i] - loaded.y[i]));
    }
    CHECK_NEAR(worst, 0.0, 1e-9);
}

TEST(FileIO, text_round_trip_preserves_shape) {
    TempFile tf("test_rt.txt");
    Pattern src = makeCircle(200, 0.5f);

    CHECK(FileSaver::saveTextFile(tf.path, src));

    Pattern loaded;
    CHECK(FileLoader::loadTextFile(tf.path, loaded));
    CHECK_EQ(loaded.size(), src.size());

    // loadTextFile normalises, so amplitude changes but the shape must not:
    // a circle of radius 0.5 becomes a circle of radius 1.
    for (size_t i = 0; i < loaded.size(); ++i) {
        double r = std::sqrt(loaded.x[i] * loaded.x[i] + loaded.y[i] * loaded.y[i]);
        CHECK_NEAR(r, 1.0, 1e-3);
    }
}

TEST(FileIO, text_loader_accepts_comments_and_both_separators) {
    TempFile tf("test_sep.txt");
    tf.write("# a comment\n0.0, 1.0\n1.0 0.0\n\n-1.0,-1.0\n");

    Pattern p;
    CHECK(FileLoader::loadTextFile(tf.path, p));
    CHECK_EQ(p.size(), size_t(3));
}

TEST(FileIO, missing_file_fails_cleanly) {
    Pattern p;
    CHECK(!FileLoader::loadTextFile("definitely-not-here.txt", p));
    CHECK(!FileLoader::loadBinaryFile("definitely-not-here.osc", p));
}

TEST(FileIO, empty_text_file_is_rejected) {
    TempFile tf("test_empty.txt");
    tf.write("# only comments\n\n");
    Pattern p;
    CHECK(!FileLoader::loadTextFile(tf.path, p));
}

TEST(FileIO, matlab_arrays_are_extracted) {
    TempFile tf("test_m.m");
    tf.write("x = [0, 1, 2, 3];\ny = [3, 2, 1, 0];\n");
    Pattern p;
    CHECK(FileLoader::loadMatlabFile(tf.path, p));
    CHECK_EQ(p.size(), size_t(4));
}

//==============================================================================
// OBJ import
//==============================================================================

namespace {
const char* kCubeObj =
    "# cube with mixed index styles\n"
    "v -1 -1 -1\nv 1 -1 -1\nv 1 1 -1\nv -1 1 -1\n"
    "v -1 -1 1\nv 1 -1 1\nv 1 1 1\nv -1 1 1\n"
    "vn 0 0 1\nvt 0 0\n"
    "f 1//1 2//1 3//1 4//1\n"
    "f 5/1/1 6/1/1 7/1/1 8/1/1\n"
    "f 1 2 6 5\nf 3 4 8 7\nf 2 3 7 6\n"
    "f -8 -5 -1 -4\n";
}

TEST(ObjLoader, cube_edges_are_deduplicated) {
    TempFile tf("test_cube.obj");
    tf.write(kCubeObj);

    ObjMesh mesh; std::string err;
    CHECK(ObjLoader::load(tf.path, mesh, err));
    CHECK_EQ(mesh.vertices.size(), size_t(8));
    CHECK_EQ(mesh.faceCount, size_t(6));
    CHECK_EQ(mesh.edges.size(), size_t(12));   // a cube has 12 unique edges
    CHECK_STR_EQ(mesh.name, "test_cube");
}

TEST(ObjLoader, model_is_centered_and_scaled) {
    TempFile tf("test_scale.obj");
    // Off-centre, oversized cube: 100..102 on every axis
    tf.write("v 100 100 100\nv 102 100 100\nv 102 102 100\nv 100 102 100\n"
             "v 100 100 102\nv 102 100 102\nv 102 102 102\nv 100 102 102\n"
             "f 1 2 3 4\nf 5 6 7 8\nf 1 2 6 5\nf 3 4 8 7\n");

    ObjMesh mesh; std::string err;
    CHECK(ObjLoader::load(tf.path, mesh, err));

    float lo = 1e9f, hi = -1e9f;
    for (const auto& v : mesh.vertices) {
        for (float c : {v.x, v.y, v.z}) { lo = std::fmin(lo, c); hi = std::fmax(hi, c); }
    }
    CHECK_NEAR(lo, -0.5, 1e-5);
    CHECK_NEAR(hi,  0.5, 1e-5);
}

TEST(ObjLoader, polyline_elements_are_traced) {
    TempFile tf("test_line.obj");
    tf.write("v 0 0 0\nv 1 0 0\nv 1 1 0\nl 1 2 3\n");
    ObjMesh mesh; std::string err;
    CHECK(ObjLoader::load(tf.path, mesh, err));
    CHECK_EQ(mesh.edges.size(), size_t(2));    // open polyline: 2 segments
    CHECK_EQ(mesh.faceCount, size_t(0));
}

TEST(ObjLoader, point_cloud_is_rejected) {
    TempFile tf("test_points.obj");
    tf.write("v 0 0 0\nv 1 1 1\n");
    ObjMesh mesh; std::string err;
    CHECK(!ObjLoader::load(tf.path, mesh, err));
    CHECK(!err.empty());
}

TEST(ObjLoader, file_without_vertices_is_rejected) {
    TempFile tf("test_novert.obj");
    tf.write("# nothing useful\n");
    ObjMesh mesh; std::string err;
    CHECK(!ObjLoader::load(tf.path, mesh, err));
}

TEST(ObjLoader, missing_file_reports_error) {
    ObjMesh mesh; std::string err;
    CHECK(!ObjLoader::load("no-such-model.obj", mesh, err));
    CHECK_STR_EQ(err, "Could not open file");
}

TEST(ObjLoader, degenerate_face_does_not_emit_self_edges) {
    TempFile tf("test_degen.obj");
    tf.write("v 5 5 5\nv 5 5 5\nf 1 2 1\n");
    ObjMesh mesh; std::string err;
    CHECK(ObjLoader::load(tf.path, mesh, err));
    for (const auto& e : mesh.edges) CHECK(e.first != e.second);
}

TEST(ObjLoader, decimate_caps_edge_count) {
    TempFile tf("test_cube2.obj");
    tf.write(kCubeObj);
    ObjMesh mesh; std::string err;
    CHECK(ObjLoader::load(tf.path, mesh, err));

    ObjLoader::decimate(mesh, 5);
    CHECK_EQ(mesh.edges.size(), size_t(5));

    // Decimating above the current count must leave the mesh untouched.
    ObjLoader::decimate(mesh, 100);
    CHECK_EQ(mesh.edges.size(), size_t(5));
}

//==============================================================================
// Sequence
//==============================================================================

TEST(Sequence, round_trip_preserves_steps) {
    TempFile tf("test_seq.oseq");

    Sequence seq;
    seq.loop = false;
    seq.crossfade = 0.25f;
    for (int i = 0; i < 3; ++i) {
        SequenceStep s;
        s.pattern = makeCircle(50 + i * 10);
        s.cycles = 100 + i;
        snprintf(s.name, sizeof(s.name), "Step %d", i + 1);
        seq.steps.push_back(std::move(s));
    }

    std::string err;
    CHECK(saveSequence(tf.path, seq, err));

    Sequence loaded;
    CHECK(loadSequence(tf.path, loaded, err));
    CHECK_EQ(loaded.size(), size_t(3));
    CHECK(loaded.loop == false);
    CHECK_NEAR(loaded.crossfade, 0.25, 1e-5);
    for (size_t i = 0; i < 3; ++i) {
        CHECK_EQ(loaded.steps[i].cycles, seq.steps[i].cycles);
        CHECK_EQ(loaded.steps[i].pattern.size(), seq.steps[i].pattern.size());
        CHECK_STR_EQ(loaded.steps[i].name, seq.steps[i].name);
    }
}

TEST(Sequence, version_1_files_still_load) {
    TempFile tf("test_v1.oseq");
    tf.write("OSEQ 1\nloop 1\nsteps 1\nstep 200 2 Legacy\n0 0\n1 1\n");

    Sequence seq; std::string err;
    CHECK(loadSequence(tf.path, seq, err));
    CHECK_EQ(seq.size(), size_t(1));
    CHECK_STR_EQ(seq.steps[0].name, "Legacy");
    CHECK_NEAR(seq.crossfade, 0.0, 1e-6);      // absent field keeps its default
}

TEST(Sequence, crlf_names_lose_the_carriage_return) {
    TempFile tf("test_crlf.oseq");
    tf.write("OSEQ 2\r\nloop 1\r\ncrossfade 0\r\nsteps 1\r\nstep 10 1 Kickdrum\r\n0 0\r\n");

    Sequence seq; std::string err;
    CHECK(loadSequence(tf.path, seq, err));
    CHECK_STR_EQ(seq.steps[0].name, "Kickdrum");
}

TEST(Sequence, bad_magic_is_rejected) {
    TempFile tf("test_bad.oseq");
    tf.write("NOPE 1\n");
    Sequence seq; std::string err;
    CHECK(!loadSequence(tf.path, seq, err));
    CHECK(!err.empty());
}

TEST(Sequence, truncated_file_is_rejected) {
    TempFile tf("test_trunc.oseq");
    tf.write("OSEQ 2\nloop 1\ncrossfade 0\nsteps 1\nstep 10 5 Short\n0 0\n1 1\n");
    Sequence seq; std::string err;
    CHECK(!loadSequence(tf.path, seq, err));
}

TEST(Sequence, absurd_step_count_is_rejected) {
    TempFile tf("test_huge.oseq");
    tf.write("OSEQ 2\nloop 1\ncrossfade 0\nsteps 999999999\n");
    Sequence seq; std::string err;
    CHECK(!loadSequence(tf.path, seq, err));
}

TEST(Sequence, saving_empty_sequence_fails) {
    Sequence seq; std::string err;
    CHECK(!saveSequence("should-not-appear.oseq", seq, err));
    std::remove("should-not-appear.oseq");
}

//==============================================================================
// Crossfade
//==============================================================================

TEST(Crossfade, endpoints_return_the_originals) {
    Pattern a = makeCircle(10, 0.5f);
    Pattern b = makeCircle(20, 1.0f);
    Pattern out;

    blendPatterns(a, b, 0.0f, out);
    CHECK_EQ(out.size(), a.size());

    blendPatterns(a, b, 1.0f, out);
    CHECK_EQ(out.size(), b.size());
}

TEST(Crossfade, midpoint_is_between_both_shapes) {
    // Two concentric circles: the halfway blend must be a circle of the mean
    // radius, which is a strong check that resampling stayed aligned.
    Pattern inner = makeCircle(64, 0.4f);
    Pattern outer = makeCircle(64, 0.8f);
    Pattern out;

    blendPatterns(inner, outer, 0.5f, out);
    CHECK_EQ(out.size(), size_t(64));
    for (size_t i = 0; i < out.size(); ++i) {
        double r = std::sqrt(out.x[i] * out.x[i] + out.y[i] * out.y[i]);
        CHECK_NEAR(r, 0.6, 1e-3);
    }
}

TEST(Crossfade, differing_sizes_take_the_larger) {
    Pattern a = makeCircle(7);
    Pattern b = makeCircle(31);
    Pattern out;
    blendPatterns(a, b, 0.5f, out);
    CHECK_EQ(out.size(), size_t(31));
}

TEST(Crossfade, empty_input_falls_back_to_the_other) {
    Pattern empty;
    Pattern circle = makeCircle(8);
    Pattern out;

    blendPatterns(empty, circle, 0.5f, out);
    CHECK_EQ(out.size(), circle.size());

    blendPatterns(circle, empty, 0.5f, out);
    CHECK_EQ(out.size(), circle.size());
}

//==============================================================================
// Recent files
//==============================================================================

TEST(RecentFiles, newest_entry_moves_to_front) {
    RecentFiles r;
    r.add("a.txt");
    r.add("b.txt");
    r.add("a.txt");                       // re-adding must not duplicate
    CHECK_EQ(r.entries().size(), size_t(2));
    CHECK_STR_EQ(r.entries()[0], "a.txt");
    CHECK_STR_EQ(r.entries()[1], "b.txt");
}

TEST(RecentFiles, list_is_capped) {
    RecentFiles r;
    for (int i = 0; i < 20; ++i) r.add("file" + std::to_string(i) + ".txt");
    CHECK_EQ(r.entries().size(), RecentFiles::MAX_ENTRIES);
    CHECK_STR_EQ(r.entries()[0], "file19.txt");     // newest first
}

TEST(RecentFiles, empty_path_is_ignored) {
    RecentFiles r;
    r.add("");
    CHECK(r.empty());
}

TEST(RecentFiles, round_trip_through_disk) {
    TempFile tf("test_recent.txt");
    RecentFiles a;
    a.add("one.txt");
    a.add("two.txt");
    CHECK(a.save(tf.path));

    RecentFiles b;
    CHECK(b.load(tf.path));
    CHECK_EQ(b.entries().size(), size_t(2));
    CHECK_STR_EQ(b.entries()[0], "two.txt");
}

TEST(RecentFiles, prune_drops_missing_entries) {
    TempFile present("test_present.txt");
    present.write("hello");

    RecentFiles r;
    r.add(present.path);
    r.add("definitely-gone.txt");
    CHECK_EQ(r.pruneMissing(), size_t(1));
    CHECK_EQ(r.entries().size(), size_t(1));
    CHECK_STR_EQ(r.entries()[0], present.path);
}

TEST(RecentFiles, display_name_is_the_file_name) {
    CHECK_STR_EQ(RecentFiles::displayName("C:/a/b/c.txt"), "c.txt");
    CHECK_STR_EQ(RecentFiles::displayName("/home/u/x.osc"), "x.osc");
    CHECK_STR_EQ(RecentFiles::displayName("bare.txt"), "bare.txt");
}

//==============================================================================
// Undo stack
//==============================================================================

TEST(UndoStack, undo_and_redo_walk_the_history) {
    UndoStack<int> stack;
    CHECK(!stack.canUndo());

    stack.push(1);                 // state before it became 2
    stack.push(2);                 // state before it became 3
    int current = 3;

    const int* u = stack.undo(current);
    CHECK(u != nullptr);
    CHECK_EQ(*u, 2);
    current = *u;

    u = stack.undo(current);
    CHECK(u != nullptr);
    CHECK_EQ(*u, 1);
    current = *u;

    CHECK(!stack.canUndo());

    const int* r = stack.redo(current);
    CHECK(r != nullptr);
    CHECK_EQ(*r, 2);
}

TEST(UndoStack, undo_on_empty_returns_null) {
    UndoStack<int> stack;
    CHECK(stack.undo(5) == nullptr);
    CHECK(stack.redo(5) == nullptr);
}

TEST(UndoStack, new_edit_clears_the_redo_branch) {
    UndoStack<int> stack;
    stack.push(1);
    const int* u = stack.undo(2);
    CHECK(u != nullptr);
    CHECK(stack.canRedo());

    stack.push(99);                // a fresh edit invalidates the redo branch
    CHECK(!stack.canRedo());
}

TEST(UndoStack, depth_is_bounded) {
    UndoStack<int> stack(4);
    for (int i = 0; i < 50; ++i) stack.push(i);
    CHECK_EQ(stack.undoDepth(), size_t(4));

    // The oldest entries were dropped, so the deepest undo is a recent value.
    const int* u = stack.undo(999);
    CHECK(u != nullptr);
    CHECK_EQ(*u, 49);
}

//==============================================================================
// Preset
//==============================================================================

TEST(Preset, key_lookup_returns_fallback_when_absent) {
    Preset p;
    CHECK_NEAR(p.valueOr("nothing", 1.5f), 1.5, 1e-6);
    p.set("thing", 2.0f);
    CHECK_NEAR(p.valueOr("thing", 1.5f), 2.0, 1e-6);
    p.set("thing", 3.0f);                       // set replaces, not duplicates
    CHECK_EQ(p.effectValues.size(), size_t(1));
    CHECK_NEAR(p.valueOr("thing", 0.0f), 3.0, 1e-6);
}

TEST(Preset, round_trip_preserves_pattern_and_values) {
    TempFile tf("test_preset.opreset");

    Preset src;
    src.name = "My Look";
    src.pattern = makeCircle(120);
    src.set("rotationSpeed", 12.5f);
    src.set("echoEnabled", 1.0f);

    std::string err;
    CHECK(savePreset(tf.path, src, err));

    Preset loaded;
    CHECK(loadPreset(tf.path, loaded, err));
    CHECK_STR_EQ(loaded.name, "My Look");
    CHECK_EQ(loaded.pattern.size(), size_t(120));
    CHECK_NEAR(loaded.valueOr("rotationSpeed", 0.0f), 12.5, 1e-4);
    CHECK_NEAR(loaded.valueOr("echoEnabled", 0.0f), 1.0, 1e-6);
}

TEST(Preset, unknown_keys_are_kept_and_ignored) {
    TempFile tf("test_future.opreset");
    tf.write("OPRESET 1\nname Future\neffects 1\nsomeFutureEffect 4.5\npoints 1\n0 0\n");

    Preset p; std::string err;
    CHECK(loadPreset(tf.path, p, err));
    CHECK_NEAR(p.valueOr("someFutureEffect", 0.0f), 4.5, 1e-4);
    // A key this build does not know must not disturb known defaults.
    CHECK_NEAR(p.valueOr("rotationSpeed", 5.0f), 5.0, 1e-6);
}

TEST(Preset, bad_magic_is_rejected) {
    TempFile tf("test_badpreset.opreset");
    tf.write("SOMETHINGELSE 1\n");
    Preset p; std::string err;
    CHECK(!loadPreset(tf.path, p, err));
}
