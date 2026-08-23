#pragma once

#include <string>
#include <utility>
#include <vector>

namespace oscilloplot {

//==============================================================================
// Wavefront OBJ wireframe mesh
//
// Only the geometry needed for XY wireframe tracing is kept: vertex positions
// and a de-duplicated edge list. Normals, texture coordinates, and materials
// are ignored.
//==============================================================================
struct ObjMesh {
    struct Vertex { float x, y, z; };

    std::vector<Vertex> vertices;
    std::vector<std::pair<int, int>> edges;

    // Source stats, for display in the UI
    std::string name;           // File stem of the loaded model
    size_t faceCount = 0;       // Faces seen in the file (before edge extraction)

    bool empty() const { return vertices.empty() || edges.empty(); }

    void clear() {
        vertices.clear();
        edges.clear();
        name.clear();
        faceCount = 0;
    }
};

class ObjLoader {
public:
    // Load a Wavefront .obj file as a wireframe mesh.
    //
    // Faces (f) are converted to their boundary edges, polylines (l) to their
    // segments; duplicate edges shared between faces are emitted once. The
    // result is centered on its bounding-box center and scaled so the largest
    // dimension spans [-0.5, 0.5], matching the built-in shapes' extents.
    //
    // On failure, `error` receives a human-readable reason and mesh is cleared.
    static bool load(const std::string& filename, ObjMesh& mesh, std::string& error);

    // Reduce an edge list to at most maxEdges by uniform stride sampling.
    // Edges follow face order, which is usually spatially coherent, so an even
    // stride keeps the wireframe spread over the whole model. Large meshes
    // would otherwise trace far too slowly to be usable.
    static void decimate(ObjMesh& mesh, size_t maxEdges);
};

} // namespace oscilloplot
