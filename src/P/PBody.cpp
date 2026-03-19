#include "PBody.hpp"

#include <algorithm>

namespace P {

    bool RigidBody::isStatic() const noexcept {
        return type == BodyType::Static;
    }

    bool RigidBody::isKinematic() const noexcept {
        return type == BodyType::Kinematic;
    }

    bool RigidBody::isDynamic() const noexcept {
        return type == BodyType::Dynamic;
    }

    void RigidBody::setMass(float newMass) noexcept {
        if (!isDynamic()) {
            mass = 0.0f;
            invMass = 0.0f;
            return;
        }

        if (newMass <= M::EPS) {
            mass = 0.0f;
            invMass = 0.0f;
            return;
        }

        mass = newMass;
        invMass = 1.0f / newMass;
    }

    void RigidBody::setInertiaTensor(const M::Matrix3x3& inertia) noexcept {
        inertiaLocal = inertia;

        if (!allowRotation || !isDynamic() || mass <= M::EPS) {
            invInertiaLocal = M::Matrix3x3::zero();
            return;
        }

        invInertiaLocal = inertia.inverted();
    }

    M::Transform3D RigidBody::transform() const noexcept {
        M::Transform3D t;
        t.position = position;
        t.rotation = rotation;
        t.scale = M::Vector3D(1.0f, 1.0f, 1.0f);
        return t;
    }

    void RigidBody::clearAccumulators() noexcept {
        forceAccum = M::Vector3D(0.0f, 0.0f, 0.0f);
        torqueAccum = M::Vector3D(0.0f, 0.0f, 0.0f);
    }

    void RigidBody::applyForce(const M::Vector3D& force) noexcept {
        if (!isDynamic()) {
            return;
        }

        forceAccum += force;
        awake = true;
    }

    void RigidBody::applyForceAtPoint(const M::Vector3D& force, const M::Point3D& worldPoint) noexcept {
        if (!isDynamic()) {
            return;
        }

        forceAccum += force;

        if (allowRotation) {
            const M::Vector3D r = worldPoint - position;
            torqueAccum += r.cross(force);
        }

        awake = true;
    }

    void RigidBody::applyTorque(const M::Vector3D& torque) noexcept {
        if (!isDynamic() || !allowRotation) {
            return;
        }

        torqueAccum += torque;
        awake = true;
    }

    void RigidBody::applyLinearImpulse(const M::Vector3D& impulse) noexcept {
        if (!isDynamic() || invMass <= M::EPS) {
            return;
        }

        linearVelocity += impulse * invMass;
        awake = true;
    }

    void RigidBody::applyAngularImpulse(const M::Vector3D& impulse) noexcept {
        if (!isDynamic() || !allowRotation) {
            return;
        }

        angularVelocity += invInertiaLocal * impulse;
        awake = true;
    }

    void RigidBody::applyImpulseAtPoint(const M::Vector3D& impulse, const M::Point3D& worldPoint) noexcept {
        if (!isDynamic() || invMass <= M::EPS) {
            return;
        }

        linearVelocity += impulse * invMass;

        if (allowRotation) {
            const M::Vector3D r = worldPoint - position;
            const M::Vector3D angularImpulse = r.cross(impulse);
            angularVelocity += invInertiaLocal * angularImpulse;
        }

        awake = true;
    }

    M::Vector3D RigidBody::velocityAtPoint(const M::Point3D& worldPoint) const noexcept {
        const M::Vector3D r = worldPoint - position;
        return linearVelocity + angularVelocity.cross(r);
    }

} // namespace P