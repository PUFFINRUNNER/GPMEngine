#include "PCollider.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace {
    [[nodiscard]] M::Vector3D rotateVector(const M::Quaternion& q, const M::Vector3D& v) noexcept {
        return q.rotatedVector(v);
    }

    [[nodiscard]] M::Point3D transformPoint(const M::Transform3D& t, const M::Point3D& p) noexcept {
        const M::Vector3D scaled(
            p.x * t.scale.x,
            p.y * t.scale.y,
            p.z * t.scale.z
        );
        return t.position + t.rotation.rotatedVector(scaled);
    }

    [[nodiscard]] float maxAbsScaleComponent(const M::Vector3D& s) noexcept {
        return std::max(std::max(std::fabs(s.x), std::fabs(s.y)), std::fabs(s.z));
    }

    ShapeType TriangleMeshCollider::type() const noexcept {
    return ShapeType::TriangleMesh;
}

M::AABB3D TriangleMeshCollider::computeAABB(const M::Transform3D& worldTransform) const noexcept {
    if (vertices.empty()) {
        return M::AABB3D();
    }

    M::Point3D first = transformPointOnly(worldTransform, vertices[0]);
    M::AABB3D box(first, first);

    for (std::size_t i = 1; i < vertices.size(); ++i) {
        box.expandToInclude(transformPointOnly(worldTransform, vertices[i]));
    }

    return box;
}

M::Matrix3x3 TriangleMeshCollider::computeLocalInertiaTensor(float /*mass*/) const noexcept {
    // Use triangle mesh colliders for static/kinematic bodies in this milestone.
    return M::Matrix3x3::zero();
}
}

namespace P {

    ShapeType SphereCollider::type() const noexcept {
        return ShapeType::Sphere;
    }

    M::AABB3D SphereCollider::computeAABB(const M::Transform3D& worldTransform) const noexcept {
        const float scaleFactor = maxAbsScaleComponent(worldTransform.scale);
        const float r = radius * scaleFactor;

        const M::Point3D center = worldTransform.position;
        return M::AABB3D(
            M::Point3D(center.x - r, center.y - r, center.z - r),
            M::Point3D(center.x + r, center.y + r, center.z + r)
        );
    }

    M::Matrix3x3 SphereCollider::computeLocalInertiaTensor(float mass) const noexcept {
        if (mass <= M::EPS || radius <= M::EPS) {
            return M::Matrix3x3::zero();
        }

        const float i = 0.4f * mass * radius * radius;
        return M::Matrix3x3(
            i, 0.0f, 0.0f,
            0.0f, i, 0.0f,
            0.0f, 0.0f, i
        );
    }

    ShapeType BoxCollider::type() const noexcept {
        return ShapeType::Box;
    }

    M::AABB3D BoxCollider::computeAABB(const M::Transform3D& worldTransform) const noexcept {
        const M::Vector3D scaledHalfExtents(
            std::fabs(halfExtents.x * worldTransform.scale.x),
            std::fabs(halfExtents.y * worldTransform.scale.y),
            std::fabs(halfExtents.z * worldTransform.scale.z)
        );

        const std::array<M::Point3D, 8> localCorners = {{
            M::Point3D(-scaledHalfExtents.x, -scaledHalfExtents.y, -scaledHalfExtents.z),
            M::Point3D( scaledHalfExtents.x, -scaledHalfExtents.y, -scaledHalfExtents.z),
            M::Point3D(-scaledHalfExtents.x,  scaledHalfExtents.y, -scaledHalfExtents.z),
            M::Point3D( scaledHalfExtents.x,  scaledHalfExtents.y, -scaledHalfExtents.z),
            M::Point3D(-scaledHalfExtents.x, -scaledHalfExtents.y,  scaledHalfExtents.z),
            M::Point3D( scaledHalfExtents.x, -scaledHalfExtents.y,  scaledHalfExtents.z),
            M::Point3D(-scaledHalfExtents.x,  scaledHalfExtents.y,  scaledHalfExtents.z),
            M::Point3D( scaledHalfExtents.x,  scaledHalfExtents.y,  scaledHalfExtents.z)
        }};

        M::Point3D first = transformPoint(worldTransform, localCorners[0]);
        M::AABB3D box(first, first);

        for (std::size_t i = 1; i < localCorners.size(); ++i) {
            box.expandToInclude(transformPoint(worldTransform, localCorners[i]));
        }

        return box;
    }

    M::Matrix3x3 BoxCollider::computeLocalInertiaTensor(float mass) const noexcept {
        if (mass <= M::EPS) {
            return M::Matrix3x3::zero();
        }

        const float wx = 2.0f * halfExtents.x;
        const float wy = 2.0f * halfExtents.y;
        const float wz = 2.0f * halfExtents.z;

        const float ix = (mass / 12.0f) * (wy * wy + wz * wz);
        const float iy = (mass / 12.0f) * (wx * wx + wz * wz);
        const float iz = (mass / 12.0f) * (wx * wx + wy * wy);

        return M::Matrix3x3(
            ix, 0.0f, 0.0f,
            0.0f, iy, 0.0f,
            0.0f, 0.0f, iz
        );
    }

    ShapeType PlaneCollider::type() const noexcept {
        return ShapeType::Plane;
    }

    M::AABB3D PlaneCollider::computeAABB(const M::Transform3D& /*worldTransform*/) const noexcept {
        const float inf = std::numeric_limits<float>::max() * 0.25f;
        return M::AABB3D(
            M::Point3D(-inf, -inf, -inf),
            M::Point3D( inf,  inf,  inf)
        );
    }

    M::Matrix3x3 PlaneCollider::computeLocalInertiaTensor(float /*mass*/) const noexcept {
        return M::Matrix3x3::zero();
    }

} // namespace P