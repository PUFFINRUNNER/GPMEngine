#include "G.hpp"

[[nodiscard]] bool P::Mesh::valid() const noexcept {
            for (const auto& f : triangles) {
                if (f.a < 0 || f.b < 0 || f.c < 0) return false;
                if (f.a >= static_cast<int>(vertices.size())) return false;
                if (f.b >= static_cast<int>(vertices.size())) return false;
                if (f.c >= static_cast<int>(vertices.size())) return false;
            }
            return true;
        }