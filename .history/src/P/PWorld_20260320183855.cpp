#include "PWorld.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "PQuery.hpp"

namespace {
    [[nodiscard]] float clampMagnitude(float value, float maxAbs) noexcept {
        if (value > maxAbs) return maxAbs;
        if (value < -maxAbs) return -maxAbs;
        return value;
    }

    [[nodiscard]] M::Vector3D clampVectorMagnitude(const M::Vector3D& v, float maxMag) noexcept {
        const float mag2 = v.magnitudeSqr();
        if (mag2 <= maxMag * maxMag || mag2 <= M::EPS) {
            return v;
        }
        return v.normalized() * maxMag;
    }

    [[nodiscard]] M::Point3D transformPoint(const M::Transform3D& t, const M::Point3D& p) noexcept {
        const M::Vector3D scaled(
            p.x * t.scale.x,
            p.y * t.scale.y,
            p.z * t.scale.z
        );
        return t.position + t.rotation.rotatedVector(scaled);
    }

    [[nodiscard]] M::Vector3D transformVector(const M::Transform3D& t, const M::Vector3D& v) noexcept {
        const M::Vector3D scaled(
            v.x * t.scale.x,
            v.y * t.scale.y,
            v.z * t.scale.z
        );
        return t.rotation.rotatedVector(scaled);
    }

    [[nodiscard]] float maxAbsScaleComponent(const M::Vector3D& s) noexcept {
        return std::max(std::max(std::fabs(s.x), std::fabs(s.y)), std::fabs(s.z));
    }

    [[nodiscard]] M::Point3D supportPointBox(const P::BoxCollider& box,
                                             const M::Transform3D& t,
                                             const M::Vector3D& dir) noexcept {
        const M::Vector3D he(
            std::fabs(box.halfExtents.x * t.scale.x),
            std::fabs(box.halfExtents.y * t.scale.y),
            std::fabs(box.halfExtents.z * t.scale.z)
        );

        const M::Point3D local(
            (dir.x >= 0.0f) ? he.x : -he.x,
            (dir.y >= 0.0f) ? he.y : -he.y,
            (dir.z >= 0.0f) ? he.z : -he.z
        );

        return transformPoint(t, local);
    }
}

namespace P {

    World::World()
        : World(WorldSettings{}) {
    }

    World::World(const WorldSettings& ws)
        : settings(ws),
          broadPhase(std::make_unique<BruteForceBroadPhase>()),
          solver(std::make_unique<SequentialImpulseSolver>()) {
    }

    RigidBody* World::createBody() {
        auto body = std::make_unique<RigidBody>();
        body->id = static_cast<std::uint32_t>(bodies.size() + 1u);
        RigidBody* raw = body.get();
        bodies.push_back(std::move(body));
        return raw;
    }

    Fixture* World::createFixture(RigidBody* body, ColliderPtr collider) {
        auto fixture = std::make_unique<Fixture>();
        fixture->id = static_cast<std::uint32_t>(fixtures.size() + 1u);
        fixture->body = body;
        fixture->collider = std::move(collider);
        fixture->collisionGroup = 1u;

        auto& lt = fixture->localTransform;
        lt.position = M::Point3D(0.0f, 0.0f, 0.0f);
        lt.rotation = M::Quaternion::identity();
        lt.scale = M::Vector3D(1.0f, 1.0f, 1.0f);

        Fixture* raw = fixture.get();
        fixtures.push_back(std::move(fixture));

        if (body && raw->collider && body->isDynamic()) {
            body->setInertiaTensor(raw->collider->computeLocalInertiaTensor(body->mass));
        }

        return raw;
    }

    void World::clear() noexcept {
        bodies.clear();
        fixtures.clear();
        broadPhasePairs.clear();
        manifolds.clear();
        constraints.clear();

        if (broadPhase) {
            broadPhase->clear();
        }
    }

    void World::step(float dt) {
        if (dt <= 0.0f) {
            return;
        }

        integrateForces(dt);
        buildBroadPhase();
        buildManifolds();
        buildConstraints();
        solveConstraints(dt);
        integrateVelocities(dt);
        stabilizeRestingBoxes(dt);
        clearAccumulators();
    }

    void World::integrateForces(float dt) noexcept {
        for (auto& bodyPtr : bodies) {
            if (!bodyPtr) {
                continue;
            }

            RigidBody& body = *bodyPtr;
            if (!body.isDynamic() || !body.awake) {
                continue;
            }

            if (body.useGravity) {
                body.linearVelocity += settings.gravity * dt;
            }

            if (body.invMass > M::EPS) {
                body.linearVelocity += body.forceAccum * (body.invMass * dt);
            }

            if (body.allowRotation) {
                body.angularVelocity += body.invInertiaLocal * (body.torqueAccum * dt);
            }

            body.linearVelocity *= std::max(0.0f, 1.0f - body.linearDamping * dt);
            body.angularVelocity *= std::max(0.0f, 1.0f - body.angularDamping * dt);

            body.linearVelocity = clampVectorMagnitude(body.linearVelocity, settings.maxLinearSpeed);
            body.angularVelocity = clampVectorMagnitude(body.angularVelocity, settings.maxAngularSpeed);
        }
    }

    void World::integrateVelocities(float dt) noexcept {
        for (auto& bodyPtr : bodies) {
            if (!bodyPtr) {
                continue;
            }

            RigidBody& body = *bodyPtr;
            if (!body.awake) {
                continue;
            }

            if (body.isDynamic() || body.isKinematic()) {
                body.position += body.linearVelocity * dt;

                if (body.allowRotation) {
                    const M::Vector3D w = body.angularVelocity;
                    if (w.magnitudeSqr() > M::EPS) {
                        const M::Quaternion omega(w.x, w.y, w.z, 0.0f);
                        M::Quaternion dq = omega * body.rotation;
                        dq *= 0.5f * dt;
                        body.rotation += dq;
                        body.rotation.normalize();
                    }
                }
            }
        }
    }

    void World::buildBroadPhase() {
        broadPhasePairs.clear();

        if (!settings.enableBroadPhase || !broadPhase) {
            return;
        }

        std::vector<Fixture*> activeFixtures;
        activeFixtures.reserve(fixtures.size());

        for (auto& fixturePtr : fixtures) {
            if (!fixturePtr) {
                continue;
            }
            if (!fixturePtr->enabled || !fixturePtr->valid()) {
                continue;
            }
            activeFixtures.push_back(fixturePtr.get());
        }

        broadPhase->build(activeFixtures);
        broadPhasePairs = broadPhase->computePairs();
    }

    void World::buildManifolds() {
        manifolds.clear();
        manifolds.reserve(broadPhasePairs.size());

        for (const auto& pair : broadPhasePairs) {
            if (!pair.valid()) {
                continue;
            }

            ContactManifold manifold = collide(*pair.a, *pair.b);
            if (manifold.valid()) {
                manifolds.push_back(manifold);
            }
        }
    }

    void World::buildConstraints() {
        constraints.clear();
        constraints.reserve(manifolds.size());

        for (const auto& manifold : manifolds) {
            if (!manifold.valid()) {
                continue;
            }

            ContactConstraint constraint;
            constraint.manifold = manifold;

            const Fixture* fa = manifold.a;
            const Fixture* fb = manifold.b;

            const float ra = fa ? fa->material.restitution : 0.0f;
            const float rb = fb ? fb->material.restitution : 0.0f;
            const float sfa = fa ? fa->material.staticFriction : 0.0f;
            const float sfb = fb ? fb->material.staticFriction : 0.0f;
            const float dfa = fa ? fa->material.dynamicFriction : 0.0f;
            const float dfb = fb ? fb->material.dynamicFriction : 0.0f;

            constraint.combinedRestitution = std::max(ra, rb);
            constraint.combinedStaticFriction = std::sqrt(std::max(0.0f, sfa * sfb));
            constraint.combinedDynamicFriction = std::sqrt(std::max(0.0f, dfa * dfb));

            constraints.push_back(constraint);
        }
    }

    void World::solveConstraints(float dt) {
        if (!solver) {
            return;
        }
        solver->solve(*this, constraints, dt, settings.solver);
    }

    void World::clearAccumulators() noexcept {
        for (auto& bodyPtr : bodies) {
            if (bodyPtr) {
                bodyPtr->clearAccumulators();
            }
        }
    }

    [[nodiscard]] bool bodyHasBoxFixture(const P::World& world,
                                        const P::RigidBody* body) noexcept {
        for (const auto& fptr : world.fixtures) {
            if (!fptr || fptr->body != body || !fptr->collider) {
                continue;
            }
            if (fptr->collider->type() == P::ShapeType::Box) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] M::Quaternion targetStableBoxOrientation(const M::Quaternion& current) noexcept {
        const M::Vector3D worldUp(0.0f, 1.0f, 0.0f);

        M::Vector3D x = current.rotatedVector(M::Vector3D(1.0f, 0.0f, 0.0f)).normalized();
        M::Vector3D y = current.rotatedVector(M::Vector3D(0.0f, 1.0f, 0.0f)).normalized();
        M::Vector3D z = current.rotatedVector(M::Vector3D(0.0f, 0.0f, 1.0f)).normalized();

        const float dx = std::fabs(x.dot(worldUp));
        const float dy = std::fabs(y.dot(worldUp));
        const float dz = std::fabs(z.dot(worldUp));

        if (dy >= dx && dy >= dz) {
            M::Vector3D targetY = (y.dot(worldUp) >= 0.0f) ? worldUp : -worldUp;
            M::Vector3D targetX = projectOnPlane(x, targetY);
            if (targetX.magnitudeSqr() <= M::EPS) {
                targetX = projectOnPlane(z, targetY);
            }
            targetX.normalize();
            M::Vector3D targetZ = targetX.cross(targetY).normalized();
            targetX = targetY.cross(targetZ).normalized();
            return quaternionFromBasis(targetX, targetY, targetZ);
        }

        if (dx >= dy && dx >= dz) {
            M::Vector3D targetX = (x.dot(worldUp) >= 0.0f) ? worldUp : -worldUp;
            M::Vector3D targetY = projectOnPlane(y, targetX);
            if (targetY.magnitudeSqr() <= M::EPS) {
                targetY = projectOnPlane(z, targetX);
            }
            targetY.normalize();
            M::Vector3D targetZ = targetX.cross(targetY).normalized();
            targetY = targetZ.cross(targetX).normalized();
            return quaternionFromBasis(targetX, targetY, targetZ);
        }

        M::Vector3D targetZ = (z.dot(worldUp) >= 0.0f) ? worldUp : -worldUp;
        M::Vector3D targetX = projectOnPlane(x, targetZ);
        if (targetX.magnitudeSqr() <= M::EPS) {
            targetX = projectOnPlane(y, targetZ);
        }
        targetX.normalize();
        M::Vector3D targetY = targetZ.cross(targetX).normalized();
        targetX = targetY.cross(targetZ).normalized();
        return quaternionFromBasis(targetX, targetY, targetZ);
    }

    ContactManifold collide(Fixture& a, Fixture& b) noexcept {
        const ShapeType ta = a.collider->type();
        const ShapeType tb = b.collider->type();

        if (ta == ShapeType::Sphere && tb == ShapeType::Sphere) {
            return collideSphereSphere(a, b);
        }

        if (ta == ShapeType::Sphere && tb == ShapeType::Plane) {
            return collideSpherePlane(a, b);
        }

        if (ta == ShapeType::Plane && tb == ShapeType::Sphere) {
            ContactManifold m = collideSpherePlane(b, a);
            if (m.valid()) {
                std::swap(m.a, m.b);
                m.normal.negate();
                for (int i = 0; i < m.pointCount; ++i) {
                    m.points[static_cast<std::size_t>(i)].normal.negate();
                }
            }
            return m;
        }

        if (ta == ShapeType::Box && tb == ShapeType::Plane) {
            return collideBoxPlane(a, b);
        }

        if (ta == ShapeType::Plane && tb == ShapeType::Box) {
            ContactManifold m = collideBoxPlane(b, a);
            if (m.valid()) {
                std::swap(m.a, m.b);
                m.normal.negate();
                for (int i = 0; i < m.pointCount; ++i) {
                    m.points[static_cast<std::size_t>(i)].normal.negate();
                }
            }
            return m;
        }

        if (ta == ShapeType::Box && tb == ShapeType::Box) {
            return collideBoxBox(a, b);
        }

        return ContactManifold{};
    }

    ContactManifold collideSphereSphere(Fixture& a, Fixture& b) noexcept {
        ContactManifold manifold{};

        auto* sa = dynamic_cast<SphereCollider*>(a.collider.get());
        auto* sb = dynamic_cast<SphereCollider*>(b.collider.get());
        if (!sa || !sb) {
            return manifold;
        }

        const M::Transform3D ta = a.worldTransform();
        const M::Transform3D tb = b.worldTransform();

        const M::Point3D ca = ta.position;
        const M::Point3D cb = tb.position;

        const float ra = sa->radius * maxAbsScaleComponent(ta.scale);
        const float rb = sb->radius * maxAbsScaleComponent(tb.scale);

        const M::Vector3D delta = cb - ca;
        const float dist2 = delta.magnitudeSqr();
        const float rsum = ra + rb;

        if (dist2 > rsum * rsum) {
            return manifold;
        }

        const float dist = std::sqrt(std::max(0.0f, dist2));
        M::Vector3D normal(1.0f, 0.0f, 0.0f);
        if (dist > M::EPS) {
            normal = delta / dist;
        }

        manifold.a = &a;
        manifold.b = &b;
        manifold.normal = normal;
        manifold.pointCount = 1;
        manifold.points[0].normal = normal;
        manifold.points[0].penetration = rsum - dist;
        manifold.points[0].position = ca + normal * (ra - 0.5f * manifold.points[0].penetration);

        return manifold;
    }

    ContactManifold collideSpherePlane(Fixture& sphereFixture,
                                       Fixture& planeFixture) noexcept {
        ContactManifold manifold{};

        auto* sphere = dynamic_cast<SphereCollider*>(sphereFixture.collider.get());
        auto* plane = dynamic_cast<PlaneCollider*>(planeFixture.collider.get());
        if (!sphere || !plane) {
            return manifold;
        }

        const M::Transform3D ts = sphereFixture.worldTransform();
        const M::Transform3D tp = planeFixture.worldTransform();

        const float radius = sphere->radius * maxAbsScaleComponent(ts.scale);

        const M::Point3D planePoint = transformPoint(tp, M::Point3D(
            plane->plane.normal.x * (-plane->plane.d),
            plane->plane.normal.y * (-plane->plane.d),
            plane->plane.normal.z * (-plane->plane.d)
        ));
        M::Vector3D planeNormal = transformVector(tp, plane->plane.normal).normalized();

        const float signedDistance = planeNormal.dot(ts.position - planePoint);
        if (signedDistance > radius) {
            return manifold;
        }

        manifold.a = &sphereFixture;
        manifold.b = &planeFixture;
        manifold.normal = planeNormal;
        manifold.pointCount = 1;
        manifold.points[0].normal = planeNormal;
        manifold.points[0].penetration = radius - signedDistance;
        manifold.points[0].position = ts.position - planeNormal * signedDistance;

        return manifold;
    }

    ContactManifold collideBoxPlane(Fixture& boxFixture,
                                    Fixture& planeFixture) noexcept {
        ContactManifold manifold{};

        auto* box = dynamic_cast<BoxCollider*>(boxFixture.collider.get());
        auto* plane = dynamic_cast<PlaneCollider*>(planeFixture.collider.get());
        if (!box || !plane) {
            return manifold;
        }

        const M::Transform3D tb = boxFixture.worldTransform();
        const M::Transform3D tp = planeFixture.worldTransform();

        const M::Point3D planePoint = transformPoint(tp, M::Point3D(
            plane->plane.normal.x * (-plane->plane.d),
            plane->plane.normal.y * (-plane->plane.d),
            plane->plane.normal.z * (-plane->plane.d)
        ));
        const M::Vector3D planeNormal = transformVector(tp, plane->plane.normal).normalized();

        const M::Point3D deepest = supportPointBox(*box, tb, -planeNormal);
        const float signedDistance = planeNormal.dot(deepest - planePoint);

        if (signedDistance > 0.0f) {
            return manifold;
        }

        manifold.a = &boxFixture;
        manifold.b = &planeFixture;
        manifold.normal = planeNormal;
        manifold.pointCount = 1;
        manifold.points[0].normal = planeNormal;
        manifold.points[0].penetration = -signedDistance;
        manifold.points[0].position = deepest - planeNormal * signedDistance;

        return manifold;
    }

    ContactManifold collideBoxBox(Fixture& a, Fixture& b) noexcept {
        ContactManifold manifold{};

        auto* ba = dynamic_cast<BoxCollider*>(a.collider.get());
        auto* bb = dynamic_cast<BoxCollider*>(b.collider.get());
        if (!ba || !bb) {
            return manifold;
        }

        const M::AABB3D aa = a.worldAABB();
        const M::AABB3D ab = b.worldAABB();

        if (!M::intersects(aa, ab)) {
            return manifold;
        }

        const float ix0 = std::max(aa.min.x, ab.min.x);
        const float ix1 = std::min(aa.max.x, ab.max.x);
        const float iy0 = std::max(aa.min.y, ab.min.y);
        const float iy1 = std::min(aa.max.y, ab.max.y);
        const float iz0 = std::max(aa.min.z, ab.min.z);
        const float iz1 = std::min(aa.max.z, ab.max.z);

        const float overlapX = ix1 - ix0;
        const float overlapY = iy1 - iy0;
        const float overlapZ = iz1 - iz0;

        if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) {
            return manifold;
        }

        float penetration = overlapX;
        int axis = 0; // 0=x, 1=y, 2=z

        M::Vector3D normal(
            (a.body && b.body && a.body->position.x < b.body->position.x) ? 1.0f : -1.0f,
            0.0f,
            0.0f
        );

        if (overlapY < penetration) {
            penetration = overlapY;
            axis = 1;
            normal = M::Vector3D(
                0.0f,
                (a.body && b.body && a.body->position.y < b.body->position.y) ? 1.0f : -1.0f,
                0.0f
            );
        }

        if (overlapZ < penetration) {
            penetration = overlapZ;
            axis = 2;
            normal = M::Vector3D(
                0.0f,
                0.0f,
                (a.body && b.body && a.body->position.z < b.body->position.z) ? 1.0f : -1.0f
            );
        }

        manifold.a = &a;
        manifold.b = &b;
        manifold.normal = normal;
        manifold.pointCount = 0;

        auto pushPoint = [&](const M::Point3D& p, float pointPenetration) {
            if (manifold.pointCount >= static_cast<int>(manifold.points.size())) {
                return;
            }

            // Reject near-duplicate points.
            for (int i = 0; i < manifold.pointCount; ++i) {
                const M::Point3D& q = manifold.points[static_cast<std::size_t>(i)].position;
                if (p.distanceSqrTo(q) < 1.0e-4f) {
                    return;
                }
            }

            ContactPoint& cp = manifold.points[static_cast<std::size_t>(manifold.pointCount)];
            cp.position = p;
            cp.normal = normal;
            cp.penetration = pointPenetration;
            cp.normalImpulse = 0.0f;
            cp.tangentImpulse1 = 0.0f;
            cp.tangentImpulse2 = 0.0f;
            ++manifold.pointCount;
        };

        if (axis == 1) {
            const float py = (normal.y > 0.0f) ? iy0 : iy1;

            pushPoint(M::Point3D(ix0, py, iz0), overlapY);
            pushPoint(M::Point3D(ix1, py, iz0), overlapY);
            pushPoint(M::Point3D(ix1, py, iz1), overlapY);
            pushPoint(M::Point3D(ix0, py, iz1), overlapY);

        } else if (axis == 0) {
            const float px = (normal.x > 0.0f) ? ix0 : ix1;

            pushPoint(M::Point3D(px, iy0, iz0), overlapX);
            pushPoint(M::Point3D(px, iy1, iz0), overlapX);
            pushPoint(M::Point3D(px, iy1, iz1), overlapX);
            pushPoint(M::Point3D(px, iy0, iz1), overlapX);

        } else {
            const float pz = (normal.z > 0.0f) ? iz0 : iz1;

            pushPoint(M::Point3D(ix0, iy0, pz), overlapZ);
            pushPoint(M::Point3D(ix1, iy0, pz), overlapZ);
            pushPoint(M::Point3D(ix1, iy1, pz), overlapZ);
            pushPoint(M::Point3D(ix0, iy1, pz), overlapZ);
        }

        if (manifold.pointCount == 0) {
            const M::Point3D ca = aa.center();
            const M::Point3D cb = ab.center();

            manifold.pointCount = 1;
            manifold.points[0].position = M::Point3D(
                (ca.x + cb.x) * 0.5f,
                (ca.y + cb.y) * 0.5f,
                (ca.z + cb.z) * 0.5f
            );
            manifold.points[0].normal = normal;
            manifold.points[0].penetration = penetration;
            manifold.points[0].normalImpulse = 0.0f;
            manifold.points[0].tangentImpulse1 = 0.0f;
            manifold.points[0].tangentImpulse2 = 0.0f;
        }

        return manifold;
    }

    [[nodiscard]] M::Vector3D projectOnPlane(const M::Vector3D& v,
                                            const M::Vector3D& n) noexcept {
        return v - n * v.dot(n);
    }

    [[nodiscard]] M::Quaternion quaternionFromBasis(const M::Vector3D& xAxis,
                                                    const M::Vector3D& yAxis,
                                                    const M::Vector3D& zAxis) noexcept {
        // Columns are the world-space directions of local X/Y/Z.
        const float m00 = xAxis.x; const float m01 = yAxis.x; const float m02 = zAxis.x;
        const float m10 = xAxis.y; const float m11 = yAxis.y; const float m12 = zAxis.y;
        const float m20 = xAxis.z; const float m21 = yAxis.z; const float m22 = zAxis.z;

        M::Quaternion q;
        const float trace = m00 + m11 + m22;

        if (trace > 0.0f) {
            const float s = std::sqrt(trace + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (m21 - m12) / s;
            q.y = (m02 - m20) / s;
            q.z = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
            q.w = (m21 - m12) / s;
            q.x = 0.25f * s;
            q.y = (m01 + m10) / s;
            q.z = (m02 + m20) / s;
        } else if (m11 > m22) {
            const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
            q.w = (m02 - m20) / s;
            q.x = (m01 + m10) / s;
            q.y = 0.25f * s;
            q.z = (m12 + m21) / s;
        } else {
            const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
            q.w = (m10 - m01) / s;
            q.x = (m02 + m20) / s;
            q.y = (m12 + m21) / s;
            q.z = 0.25f * s;
        }

        q.normalize();
        return q;
    }
    
    void World::stabilizeRestingBoxes(float dt) noexcept {
        const M::Vector3D worldUp(0.0f, 1.0f, 0.0f);

        for (auto& bodyPtr : bodies) {
            if (!bodyPtr) {
                continue;
            }

            RigidBody& body = *bodyPtr;
            if (!body.isDynamic()) {
                continue;
            }

            if (!bodyHasBoxFixture(*this, &body)) {
                continue;
            }

            const float linearSpeed2 = body.linearVelocity.magnitudeSqr();
            const float angularSpeed2 = body.angularVelocity.magnitudeSqr();

            float sumX = 0.0f;
            float sumY = 0.0f;
            float sumZ = 0.0f;
            int supportPointCount = 0;
            bool supportedFromBelow = false;

            for (const auto& manifold : manifolds) {
                if (!manifold.valid()) {
                    continue;
                }

                M::Vector3D supportNormal;
                bool bodyInManifold = false;

                if (manifold.a && manifold.a->body == &body) {
                    supportNormal = -manifold.normal;
                    bodyInManifold = true;
                } else if (manifold.b && manifold.b->body == &body) {
                    supportNormal = manifold.normal;
                    bodyInManifold = true;
                }

                if (!bodyInManifold) {
                    continue;
                }

                if (supportNormal.dot(worldUp) < 0.55f) {
                    continue;
                }

                supportedFromBelow = true;

                for (int i = 0; i < manifold.pointCount; ++i) {
                    const M::Point3D& p = manifold.points[static_cast<std::size_t>(i)].position;
                    sumX += p.x;
                    sumY += p.y;
                    sumZ += p.z;
                    ++supportPointCount;
                }
            }

            if (!supportedFromBelow || supportPointCount == 0) {
                continue;
            }

            const M::Point3D supportCenter(
                sumX / static_cast<float>(supportPointCount),
                sumY / static_cast<float>(supportPointCount),
                sumZ / static_cast<float>(supportPointCount)
            );

            // If the box is nearly resting but its center of mass is not centered over support,
            // add a restoring torque so it falls flat instead of balancing unrealistically on an edge.
            if (linearSpeed2 < settings.restingLinearThreshold * settings.restingLinearThreshold) {
                const M::Vector3D r = body.position - supportCenter;
                const M::Vector3D horizontalOffset(r.x, 0.0f, r.z);

                if (horizontalOffset.magnitudeSqr() > 1.0e-5f) {
                    const M::Vector3D gravityDir(0.0f, -1.0f, 0.0f);
                    const M::Vector3D settleTorque =
                        horizontalOffset.cross(gravityDir) *
                        (settings.settleAssistTorque * std::max(body.mass, 0.1f));

                    if (body.allowRotation) {
                        body.angularVelocity += (body.invInertiaLocal * settleTorque) * dt;
                        body.awake = true;
                    }
                }
            }

            // Gentle snap to a face-stable orientation once the body is already almost settled.
            if (linearSpeed2 < settings.restingLinearThreshold * settings.restingLinearThreshold &&
                angularSpeed2 < settings.restingAngularThreshold * settings.restingAngularThreshold) {

                const M::Vector3D xAxis = body.rotation.rotatedVector(M::Vector3D(1.0f, 0.0f, 0.0f)).normalized();
                const M::Vector3D yAxis = body.rotation.rotatedVector(M::Vector3D(0.0f, 1.0f, 0.0f)).normalized();
                const M::Vector3D zAxis = body.rotation.rotatedVector(M::Vector3D(0.0f, 0.0f, 1.0f)).normalized();

                const float bestDot = std::max(
                    std::fabs(xAxis.dot(worldUp)),
                    std::max(std::fabs(yAxis.dot(worldUp)), std::fabs(zAxis.dot(worldUp)))
                );

                if (bestDot >= settings.settleSnapDot) {
                    body.rotation = targetStableBoxOrientation(body.rotation);
                    body.rotation.normalize();

                    // Remove tiny residual motion so it does not buzz on an edge forever.
                    body.angularVelocity *= 0.2f;
                    if (body.angularVelocity.magnitudeSqr() < 0.01f) {
                        body.angularVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
                    }

                    body.linearVelocity *= 0.92f;
                    if (body.linearVelocity.magnitudeSqr() < 0.0025f) {
                        body.linearVelocity = M::Vector3D(0.0f, 0.0f, 0.0f);
                    }
                }
            }
        }
    }

} // namespace P