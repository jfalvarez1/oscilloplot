#include "obj_loader.hpp"

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace oscilloplot {

namespace {

// Pack a normalized (low, high) index pair into one key for de-duplication.
inline uint64_t edgeKey(int a, int b) {
    if (a > b) { int t = a; a = b; b = t; }
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
           static_cast<uint32_t>(b);
}

// Parse one face/line element ("12", "12/3", "12//4", "12/3/4") into a
// 0-based vertex index. OBJ indices are 1-based; negative values count back
// from the most recently defined vertex. Returns false if unusable.
bool parseIndex(const std::string& token, size_t vertexCount, int& out) {
    if (token.empty()) return false;

    // Only the first component (the position index) matters here.
    size_t slash = token.find('/');
    std::string head = (slash == std::string::npos) ? token : token.substr(0, slash);
    if (head.empty()) return false;

    char* end = nullptr;
    long idx = std::strtol(head.c_str(), &end, 10);
    if (end == head.c_str()) return false;

    if (idx > 0) {
        out = static_cast<int>(idx - 1);
    } else if (idx < 0) {
        out = static_cast<int>(static_cast<long>(vertexCount) + idx);
    } else {
        return false;  // Index 0 is not valid in OBJ
    }

    return out >= 0 && static_cast<size_t>(out) < vertexCount;
}

std::string fileStem(const std::string& path) {
    size_t slash = path.find_last_of("/\\");
    std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
    size_t dot = base.find_last_of('.');
    return (dot == std::string::npos) ? base : base.substr(0, dot);
}

} // namespace

bool ObjLoader::load(const std::string& filename, ObjMesh& mesh, std::string& error) {
    mesh.clear();
    error.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        error = "Could not open file";
        return false;
    }

    std::unordered_set<uint64_t> seenEdges;
    std::vector<int> face;   // Reused per face to avoid reallocating
    std::string line;

    while (std::getline(file, line)) {
        // Trim trailing CR from CRLF files read in text mode on non-Windows
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string tag;
        iss >> tag;

        if (tag == "v") {
            ObjMesh::Vertex v{0.0f, 0.0f, 0.0f};
            if (iss >> v.x >> v.y >> v.z) {
                mesh.vertices.push_back(v);
            }
        } else if (tag == "f" || tag == "l") {
            face.clear();
            std::string token;
            while (iss >> token) {
                int idx = 0;
                if (parseIndex(token, mesh.vertices.size(), idx)) {
                    face.push_back(idx);
                }
            }
            if (face.size() < 2) continue;

            if (tag == "f") mesh.faceCount++;

            // A face is a closed loop; a polyline (l) is open.
            size_t segments = (tag == "f") ? face.size() : face.size() - 1;
            for (size_t i = 0; i < segments; ++i) {
                int a = face[i];
                int b = face[(i + 1) % face.size()];
                if (a == b) continue;
                if (seenEdges.insert(edgeKey(a, b)).second) {
                    mesh.edges.emplace_back(a, b);
                }
            }
        }
        // v/vt/vn, mtllib, usemtl, o, g, s are all ignored.
    }

    if (mesh.vertices.empty()) {
        error = "No vertices found";
        mesh.clear();
        return false;
    }
    if (mesh.edges.empty()) {
        error = "No faces or lines found (point cloud is not traceable)";
        mesh.clear();
        return false;
    }

    // Center on the bounding-box center and scale the largest dimension to
    // span [-0.5, 0.5], matching the extents of the built-in shapes.
    float minX = mesh.vertices[0].x, maxX = minX;
    float minY = mesh.vertices[0].y, maxY = minY;
    float minZ = mesh.vertices[0].z, maxZ = minZ;
    for (const auto& v : mesh.vertices) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.y > maxY) maxY = v.y;
        if (v.z < minZ) minZ = v.z;
        if (v.z > maxZ) maxZ = v.z;
    }

    float cx = (minX + maxX) * 0.5f;
    float cy = (minY + maxY) * 0.5f;
    float cz = (minZ + maxZ) * 0.5f;

    float extent = maxX - minX;
    if (maxY - minY > extent) extent = maxY - minY;
    if (maxZ - minZ > extent) extent = maxZ - minZ;

    float scale = (extent > 0.0f) ? (1.0f / extent) : 1.0f;

    for (auto& v : mesh.vertices) {
        v.x = (v.x - cx) * scale;
        v.y = (v.y - cy) * scale;
        v.z = (v.z - cz) * scale;
    }

    mesh.name = fileStem(filename);
    return true;
}

void ObjLoader::decimate(ObjMesh& mesh, size_t maxEdges) {
    if (maxEdges == 0 || mesh.edges.size() <= maxEdges) return;

    // Even stride over the original order keeps coverage across the model.
    std::vector<std::pair<int, int>> kept;
    kept.reserve(maxEdges);

    double stride = static_cast<double>(mesh.edges.size()) / static_cast<double>(maxEdges);
    for (size_t i = 0; i < maxEdges; ++i) {
        size_t src = static_cast<size_t>(static_cast<double>(i) * stride);
        if (src >= mesh.edges.size()) break;
        kept.push_back(mesh.edges[src]);
    }

    mesh.edges.swap(kept);
}

} // namespace oscilloplot
