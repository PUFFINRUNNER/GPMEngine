#ifndef P_BODY_HPP
#define P_BODY_HPP

#include <cstdint>
#include "M.hpp"

namespace P {

    enum class BodyType {
        Static,
        Kinematic,
        Dynamic
    };

    struct PhysicsMaterial {
        float restitution = 0.2f;
        float staticFriction = 0.6f;
        float dynamicFriction = 0.4f;
        float density = 1.0f;
    };

    class RigidBody {
    public:
        std::uint32_t id = 0;
        BodyType type = BodyType::Dynamic;

        M::Point3D position;
        M::Quaternion rotation;

        M::Vector3D linearVelocity;
        M::Vector3D angularVelocity;

        M::Vector3D forceAccum;
        M::Vector3D torqueAccum;

        float mass = 1.0f;
        float invMass = 1.0f;

        M::Matrix3x3 inertiaLocal;
        M::Matrix3x3 invInertiaLocal;

        float linearDamping = 0.01f;
        float angularDamping = 0.01f;

        bool useGravity = true;
        bool awake = true;
        bool canSleep = true;
        bool allowRotation = true;

        RigidBody() = default;

        [[nodiscard]] bool isStatic() const noexcept;
        [[nodiscard]] bool isKinematic() const noexcept;
        [[nodiscard]] bool isDynamic() const noexcept;

        void setMass(float newMass) noexcept;
        void setInertiaTensor(const M::Matrix3x3& inertia) noexcept;

        [[nodiscard]] M::Transform3D transform() const noexcept;

        void clearAccumulators() noexcept;

        void applyForce(const M::Vector3D& force) noexcept;
        void applyForceAtPoint(const M::Vector3D& force, const M::Point3D& worldPoint) noexcept;

        void applyTorque(const M::Vector3D& torque) noexcept;

        void applyLinearImpulse(const M::Vector3D& impulse) noexcept;
        void applyAngularImpulse(const M::Vector3D& impulse) noexcept;
        void applyImpulseAtPoint(const M::Vector3D& impulse, const M::Point3D& worldPoint) noexcept;

        [[nodiscard]] M::Vector3D velocityAtPoint(const M::Point3D& worldPoint) const noexcept;
    };

} // namespace P

#endif