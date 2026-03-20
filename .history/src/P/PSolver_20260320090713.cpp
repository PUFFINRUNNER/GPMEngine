#include "PSolver.hpp"

#include <algorithm>
#include <cmath>

#include "PWorld.hpp"
#include "PFixture.hpp"
#include "PBody.hpp"

namespace {
    [[nodiscard]] float clampNonNegative(float x) noexcept {
        return (x < 0.0f) ? 0.0f : x;
    }

    [[nodiscard]] float combineRestitution(float a, float b) noexcept {
        return std::max(a, b);
    }

    [[nodiscard]] float combineStaticFriction(float a, float b) noexcept {
        return std::sqrt(std::max(0.0f, a * b));
    }

    [[nodiscard]] float combineDynamicFriction(float a, float b) noexcept {
        return std::sqrt(std::max(0.0f, a * b));
    }

    [[nodiscard]] M::Vector3D computeTangent(const M::Vector3D& v, const M::Vector3D& n) noexcept {
        M::Vector3D t = v - n * v.dot(n);
        if (t.magnitudeSqr() > M::EPS) {
            return t.normalized();
        }

        // Stable fallback tangent basis if tangential velocity is tiny.
        if (std::fabs(n.x) < 0.577f) {
            return n.cross(M::Vector3D(1.0f, 0.0f, 0.0f)).normalized();
        }
        return n.cross(M::Vector3D(0.0f, 1.0f, 0.0f)).normalized();
    }

    [[nodiscard]] M::Vector3D invInertiaWorldMul(const P::RigidBody& body,
                                                 const M::Vector3D& v) noexcept {
        if (!body.allowRotation || !body.isDynamic()) {
            return M::Vector3D(0.0f, 0.0f, 0.0f);
        }

        const M::Matrix4x4 rot4 = body.rotation.toMatrix4x4();
        const M::Matrix3x3 R(
            rot4.m[0][0], rot4.m[0][1], rot4.m[0][2],
            rot4.m[1][0], rot4.m[1][1], rot4.m[1][2],
            rot4.m[2][0], rot4.m[2][1], rot4.m[2][2]
        );

        const M::Matrix3x3 worldInvInertia = R * body.invInertiaLocal * R.transposed();
        return worldInvInertia * v;
    }

    [[nodiscard]] float computeNormalMass(const P::RigidBody* a,
                                          const P::RigidBody* b,
                                          const M::Vector3D& ra,
                                          const M::Vector3D& rb,
                                          const M::Vector3D& n) noexcept {
        float k = 0.0f;

        if (a && a->isDynamic()) {
            k += a->invMass;
            if (a->allowRotation) {
                const M::Vector3D rn = ra.cross(n);
                const M::Vector3D ang = invInertiaWorldMul(*a, rn).cross(ra);
                k += n.dot(ang);
            }
        }

        if (b && b->isDynamic()) {
            k += b->invMass;
            if (b->allowRotation) {
                const M::Vector3D rn = rb.cross(n);
                const M::Vector3D ang = invInertiaWorldMul(*b, rn).cross(rb);
                k += n.dot(ang);
            }
        }

        if (k <= M::EPS) {
            return 0.0f;
        }
        return 1.0f / k;
    }

    [[nodiscard]] float computeTangentMass(const P::RigidBody* a,
                                           const P::RigidBody* b,
                                           const M::Vector3D& ra,
                                           const M::Vector3D& rb,
                                           const M::Vector3D& t) noexcept {
        float k = 0.0f;

        if (a && a->isDynamic()) {
            k += a->invMass;
            if (a->allowRotation) {
                const M::Vector3D rt = ra.cross(t);
                const M::Vector3D ang = invInertiaWorldMul(*a, rt).cross(ra);
                k += t.dot(ang);
            }
        }

        if (b && b->isDynamic()) {
            k += b->invMass;
            if (b->allowRotation) {
                const M::Vector3D rt = rb.cross(t);
                const M::Vector3D ang = invInertiaWorldMul(*b, rt).cross(rb);
                k += t.dot(ang);
            }
        }

        if (k <= M::EPS) {
            return 0.0f;
        }
        return 1.0f / k;
    }

    void applyImpulsePair(P::RigidBody* a,
                          P::RigidBody* b,
                          const M::Vector3D& ra,
                          const M::Vector3D& rb,
                          const M::Vector3D& impulse) noexcept {
        if (a && a->isDynamic()) {
            a->linearVelocity -= impulse * a->invMass;
            if (a->allowRotation) {
                a->angularVelocity -= invInertiaWorldMul(*a, ra.cross(impulse));
            }
            a->awake = true;
        }

        if (b && b->isDynamic()) {
            b->linearVelocity += impulse * b->invMass;
            if (b->allowRotation) {
                b->angularVelocity += invInertiaWorldMul(*b, rb.cross(impulse));
            }
            b->awake = true;
        }
    }

    [[nodiscard]] M::Vector3D relativeVelocityAtPoint(const P::RigidBody* a,
                                                      const P::RigidBody* b,
                                                      const M::Point3D& p) noexcept {
        M::Vector3D va(0.0f, 0.0f, 0.0f);
        M::Vector3D vb(0.0f, 0.0f, 0.0f);

        if (a) {
            va = a->velocityAtPoint(p);
        }
        if (b) {
            vb = b->velocityAtPoint(p);
        }

        return vb - va;
    }

    void solveVelocityOnePoint(P::ContactConstraint& constraint,
                               P::ContactPoint& cp,
                               bool enableFriction,
                               bool enableRestitution) noexcept {
        P::RigidBody* bodyA = constraint.manifold.a ? constraint.manifold.a->body : nullptr;
        P::RigidBody* bodyB = constraint.manifold.b ? constraint.manifold.b->body : nullptr;

        if ((!bodyA || !bodyA->isDynamic()) && (!bodyB || !bodyB->isDynamic())) {
            return;
        }

        const M::Vector3D n = constraint.manifold.normal.normalized();

        const M::Point3D centerA = bodyA ? bodyA->position : M::Point3D();
        const M::Point3D centerB = bodyB ? bodyB->position : M::Point3D();

        const M::Vector3D ra = cp.position - centerA;
        const M::Vector3D rb = cp.position - centerB;

        const M::Vector3D rv = relativeVelocityAtPoint(bodyA, bodyB, cp.position);
        const float vn = rv.dot(n);

        float restitution = enableRestitution ? constraint.combinedRestitution : 0.0f;
        if (vn > -0.5f) {
            restitution = 0.0f;
        }

        const float normalMass = computeNormalMass(bodyA, bodyB, ra, rb, n);
        if (normalMass > 0.0f) {
            const float j = -(1.0f + restitution) * vn * normalMass;
            if (j > 0.0f) {
                const M::Vector3D impulse = n * j;
                applyImpulsePair(bodyA, bodyB, ra, rb, impulse);
                cp.normalImpulse += j;
            }
        }

        if (!enableFriction) {
            return;
        }

        const M::Vector3D rv2 = relativeVelocityAtPoint(bodyA, bodyB, cp.position);
        const M::Vector3D t = computeTangent(rv2, n);
        if (t.magnitudeSqr() <= M::EPS) {
            return;
        }

        const float tangentMass = computeTangentMass(bodyA, bodyB, ra, rb, t);
        if (tangentMass <= 0.0f) {
            return;
        }

        const float vt = rv2.dot(t);
        float jt = -vt * tangentMass;

        const float maxFriction = constraint.combinedDynamicFriction * cp.normalImpulse;
        jt = std::clamp(jt, -maxFriction, maxFriction);

        const M::Vector3D frictionImpulse = t * jt;
        applyImpulsePair(bodyA, bodyB, ra, rb, frictionImpulse);
        cp.tangentImpulse1 += jt;
    }

    void positionalCorrection(P::ContactConstraint& constraint,
                              float baumgarte,
                              float penetrationSlop) noexcept {
        P::RigidBody* bodyA = constraint.manifold.a ? constraint.manifold.a->body : nullptr;
        P::RigidBody* bodyB = constraint.manifold.b ? constraint.manifold.b->body : nullptr;

        if ((!bodyA || !bodyA->isDynamic()) && (!bodyB || !bodyB->isDynamic())) {
            return;
        }

        const M::Vector3D n = constraint.manifold.normal.normalized();

        for (int i = 0; i < constraint.manifold.pointCount; ++i) {
            const P::ContactPoint& cp = constraint.manifold.points[static_cast<std::size_t>(i)];
            const float penetration = cp.penetration;
            const float depth = clampNonNegative(penetration - penetrationSlop);
            if (depth <= 0.0f) {
                continue;
            }

            const float invMassA = (bodyA && bodyA->isDynamic()) ? bodyA->invMass : 0.0f;
            const float invMassB = (bodyB && bodyB->isDynamic()) ? bodyB->invMass : 0.0f;
            const float invMassSum = invMassA + invMassB;
            if (invMassSum <= M::EPS) {
                continue;
            }

            const M::Vector3D correction = n * ((baumgarte * depth) / invMassSum);

            if (bodyA && bodyA->isDynamic()) {
                bodyA->position -= correction * invMassA;
                bodyA->awake = true;
            }

            if (bodyB && bodyB->isDynamic()) {
                bodyB->position += correction * invMassB;
                bodyB->awake = true;
            }
        }
    }
}

namespace P {

    void SequentialImpulseSolver::solve(
        World& /*world*/,
        std::vector<ContactConstraint>& constraints,
        float /*dt*/,
        const SolverSettings& settings) {

        for (auto& constraint : constraints) {
            if (!constraint.manifold.valid()) {
                continue;
            }

            Fixture* fa = constraint.manifold.a;
            Fixture* fb = constraint.manifold.b;

            const float ra = fa ? fa->material.restitution : 0.0f;
            const float rb = fb ? fb->material.restitution : 0.0f;
            const float sfa = fa ? fa->material.staticFriction : 0.0f;
            const float sfb = fb ? fb->material.staticFriction : 0.0f;
            const float dfa = fa ? fa->material.dynamicFriction : 0.0f;
            const float dfb = fb ? fb->material.dynamicFriction : 0.0f;

            constraint.combinedRestitution = combineRestitution(ra, rb);
            constraint.combinedStaticFriction = combineStaticFriction(sfa, sfb);
            constraint.combinedDynamicFriction = combineDynamicFriction(dfa, dfb);
        }

        for (int iter = 0; iter < settings.velocityIterations; ++iter) {
            for (auto& constraint : constraints) {
                if (!constraint.manifold.valid()) {
                    continue;
                }

                for (int i = 0; i < constraint.manifold.pointCount; ++i) {
                    solveVelocityOnePoint(
                        constraint,
                        constraint.manifold.points[static_cast<std::size_t>(i)],
                        settings.enableFriction,
                        settings.enableRestitution
                    );
                }
            }
        }

        for (int iter = 0; iter < settings.positionIterations; ++iter) {
            for (auto& constraint : constraints) {
                if (!constraint.manifold.valid()) {
                    continue;
                }

                positionalCorrection(
                    constraint,
                    settings.baumgarte,
                    settings.penetrationSlop
                );
            }
        }
    }

} // namespace P