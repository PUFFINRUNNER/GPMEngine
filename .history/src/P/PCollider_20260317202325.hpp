#ifndef P_COLLIDER_HPP
#define P_COLLIDER_HPP

#include <memory>
#include <vector>
#include "M.hpp"

namespace P {

    enum class ShapeType {
        Sphere,
        Box,
        Plane
    };

    class Collider {
    public:
        virtual ~Collider() = default;

        [[nodiscard]] virtual ShapeType type() const noexcept = 0;

        [[nodiscard]] virtual M::AABB3D computeAABB(
            const M::Transform3D& worldTransform) const noexcept = 0;

        [[nodiscard]] virtual M::Matrix3x3 computeLocalInertiaTensor(float mass) const noexcept = 0;
    };

    class SphereCollider final : public Collider {
    public:
        float radius = 0.5f;

        SphereCollider() = default;
        explicit SphereCollider(float r) noexcept : radius(r) {}

        [[nodiscard]] ShapeType type() const noexcept override;
        [[nodiscard]] M::AABB3D computeAABB(
            const M::Transform3D& worldTransform) const noexcept override;
        [[nodiscard]] M::Matrix3x3 computeLocalInertiaTensor(float mass) const noexcept override;
    };

    class BoxCollider final : public Collider {
    public:
        M::Vector3D halfExtents {0.5f, 0.5f, 0.5f};

        BoxCollider() = default;
        explicit BoxCollider(const M::Vector3D& he) noexcept : halfExtents(he) {}

        [[nodiscard]] ShapeType type() const noexcept override;
        [[nodiscard]] M::AABB3D computeAABB(
            const M::Transform3D& worldTransform) const noexcept override;
        [[nodiscard]] M::Matrix3x3 computeLocalInertiaTensor(float mass) const noexcept override;
    };

    class PlaneCollider final : public Collider {
    public:
        M::Plane3D plane;

        PlaneCollider() = default;
        explicit PlaneCollider(const M::Plane3D& p) noexcept : plane(p) {}

        [[nodiscard]] ShapeType type() const noexcept override;
        [[nodiscard]] M::AABB3D computeAABB(
            const M::Transform3D& worldTransform) const noexcept override;
        [[nodiscard]] M::Matrix3x3 computeLocalInertiaTensor(float mass) const noexcept override;
    };

    using ColliderPtr = std::shared_ptr<Collider>;

} // namespace P

#endif