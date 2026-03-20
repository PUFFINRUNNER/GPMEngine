#ifndef G_MODEL_HPP
#define G_MODEL_HPP

#include <memory>
#include <vector>

#include "G.hpp"

namespace G {

    struct ModelPart {
        std::shared_ptr<Mesh> mesh;
        Material3D material;
    };

    class Model {
    public:
        std::vector<ModelPart> parts;

        [[nodiscard]] bool empty() const noexcept {
            return parts.empty();
        }

        void clear() noexcept {
            parts.clear();
        }
    };

} // namespace G

#endif