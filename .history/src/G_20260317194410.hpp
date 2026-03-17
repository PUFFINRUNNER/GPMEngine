#ifndef G_HPP
#define G_HPP

#include <vector>
#include "M.hpp"
namespace G {
    struct Vertex {
        M::Point3D position;
        M::Vector3D normal;   // optional at first
        M::Vector2D uv;
    };

    struct Triangle {
        int a, b, c;       // indices into vertices
    };

    class Mesh {
      public:
        std::vector<Vertex> vertices;
        std::vector<Triangle> triangles;

        [[nodiscard]] bool empty() const noexcept {
            return vertices.empty() || triangles.empty();
        }

        void clear() {
            vertices.clear();
            triangles.clear();
        }

        [[nodiscard]] bool valid() const noexcept {
            
        }
    };
}

#endif