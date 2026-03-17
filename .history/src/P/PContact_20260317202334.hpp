#ifndef P_CONTACT_HPP
#define P_CONTACT_HPP

#include <array>
#include <cstdint>
#include "M.hpp"

namespace P {

    class Fixture;

    struct ContactPoint {
        M::Point3D position;
        M::Vector3D normal;
        float penetration = 0.0f;

        float normalImpulse = 0.0f;
        float tangentImpulse1 = 0.0f;
        float tangentImpulse2 = 0.0f;
    };

    struct ContactManifold {
        Fixture* a = nullptr;
        Fixture* b = nullptr;

        M::Vector3D normal;
        int pointCount = 0;
        std::array<ContactPoint, 4> points {};

        [[nodiscard]] bool valid() const noexcept;
        void clear() noexcept;
    };

    struct ContactPair {
        Fixture* a = nullptr;
        Fixture* b = nullptr;

        [[nodiscard]] bool valid() const noexcept;
    };

    struct ContactConstraint {
        ContactManifold manifold;

        float combinedRestitution = 0.0f;
        float combinedStaticFriction = 0.0f;
        float combinedDynamicFriction = 0.0f;
    };

} // namespace P

#endif