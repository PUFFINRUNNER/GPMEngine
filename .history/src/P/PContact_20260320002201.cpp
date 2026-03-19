#include "PContact.hpp"

namespace P {

    bool ContactManifold::valid() const noexcept {
        if (a == nullptr || b == nullptr) {
            return false;
        }
        if (pointCount <= 0 || pointCount > static_cast<int>(points.size())) {
            return false;
        }
        return true;
    }

    void ContactManifold::clear() noexcept {
        a = nullptr;
        b = nullptr;
        normal = M::Vector3D(0.0f, 0.0f, 0.0f);
        pointCount = 0;

        for (auto& point : points) {
            point = ContactPoint{};
        }
    }

    bool ContactPair::valid() const noexcept {
        return a != nullptr && b != nullptr && a != b;
    }

} // namespace P