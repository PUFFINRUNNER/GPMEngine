#ifndef P_QUERY_HPP
#define P_QUERY_HPP

#include "M.hpp"

namespace P {

    class Fixture;
    class RigidBody;
    class World;

    struct RaycastHit {
        bool hit = false;
        float t = 0.0f;

        Fixture* fixture = nullptr;
        RigidBody* body = nullptr;

        M::Point3D point;
        M::Vector3D normal;
    };

    [[nodiscard]] RaycastHit raycast(
        const World& world,
        const M::Ray3D& ray,
        float maxDistance = 1.0e30f) noexcept;

    [[nodiscard]] bool intersects(
        const Fixture& a,
        const Fixture& b) noexcept;

} // namespace P

#endif