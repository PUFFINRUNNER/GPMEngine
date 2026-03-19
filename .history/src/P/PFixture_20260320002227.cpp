#include "PFixture.hpp"

namespace P {

    bool Fixture::valid() const noexcept {
        return body != nullptr && static_cast<bool>(collider);
    }

    M::Transform3D Fixture::worldTransform() const noexcept {
        if (!body) {
            return localTransform;
        }

        const M::Transform3D bodyTransform = body->transform();

        M::Transform3D out;
        out.position = bodyTransform.appliedTo(localTransform.position);
        out.rotation = bodyTransform.rotation * localTransform.rotation;
        out.scale = M::Vector3D(
            bodyTransform.scale.x * localTransform.scale.x,
            bodyTransform.scale.y * localTransform.scale.y,
            bodyTransform.scale.z * localTransform.scale.z
        );
        return out;
    }

    M::AABB3D Fixture::worldAABB() const noexcept {
        if (!collider) {
            return M::AABB3D();
        }
        return collider->computeAABB(worldTransform());
    }

    bool Fixture::canCollideWith(const Fixture& other) const noexcept {
        if (this == &other) {
            return false;
        }
        if (!enabled || !other.enabled) {
            return false;
        }
        if (!valid() || !other.valid()) {
            return false;
        }

        // Group/mask filtering
        if ((collisionMask & other.collisionGroup) == 0u &&
            (other.collisionMask & collisionGroup) == 0u) {
            return false;
        }

        // Fixtures on the same body should not self-collide in this milestone
        if (body != nullptr && body == other.body) {
            return false;
        }

        return true;
    }

} // namespace P