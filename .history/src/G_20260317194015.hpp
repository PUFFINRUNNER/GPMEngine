#ifndef G_HPP
#define G_HPP

#include <vector>
#include "M.hpp"
namespace G {
    struct Vertex {
        M::Point3D position;
        M::Vector3D normal;   // optional at first
        M::Vector2D uv;        // add later if needed
    };

    struct Triangle {
        int a, b, c;       // indices into vertices
    };

    class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<Triable> faces;

    [[nodiscard]] bool empty() const noexcept {
        return vertices.empty() || faces.empty();
    }

    void clear() {
        vertices.clear();
        faces.clear();
    }

    [[nodiscard]] bool valid() const noexcept {
        for (const auto& f : faces) {
            if (f.a < 0 || f.b < 0 || f.c < 0) return false;
            if (f.a >= static_cast<int>(vertices.size())) return false;
            if (f.b >= static_cast<int>(vertices.size())) return false;
            if (f.c >= static_cast<int>(vertices.size())) return false;
        }
        return true;
    }
};
}

#endif