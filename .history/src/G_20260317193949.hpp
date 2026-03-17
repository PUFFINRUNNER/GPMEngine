#ifndef G_HPP
#define G_HPP

#include <vector>
#include "M.hpp"
namespace G {
    class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<Face> faces;

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

#endif