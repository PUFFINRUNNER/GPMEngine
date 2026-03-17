#ifndef P_FIXTURE_HPP
#define P_FIXTURE_HPP

#include <cstdint>
#include <memory>
#include "M.hpp"
#include "PBody.hpp"
#include "PCollider.hpp"

namespace P {

    class Fixture {
    public:
        std::uint32_t id = 0;

        RigidBody* body = nullptr;
        ColliderPtr collider;

        PhysicsMaterial material;
        M::Transform3D localTransform;

        bool isTrigger = false;
        bool enabled = true;

        std::uint32_t collisionGroup = 0;
        std::uint32_t collisionMask = 0xFFFFFFFFu;

        Fixture() = default;

        [[nodiscard]] bool valid() const noexcept;

        [[nodiscard]] M::Transform3D worldTransform() const noexcept;
        [[nodiscard]] M::AABB3D worldAABB() const noexcept;

        [[nodiscard]] bool canCollideWith(const Fixture& other) const noexcept;
    };

} // namespace P

#endif