#include "PQuery.hpp"

#include <limits>
#include <cmath>

#include "PWorld.hpp"
#include "PFixture.hpp"
#include "PCollider.hpp"
#include "PBody.hpp"

namespace {
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

    [[nodiscard]] bool raycastSphere(const M::Ray3D& ray,
                                     float maxDistance,
                                     const P::Fixture& fixture,
                                     P::RaycastHit& out) noexcept {
        const auto* sphere = dynamic_cast<const P::SphereCollider*>(fixture.collider.get());
        if (!sphere) {
            return false;
        }

        const M::Transform3D wt = fixture.worldTransform();
        const M::Point3D center = wt.position;

        const float sx = std::fabs(wt.scale.x);
        const float sy = std::fabs(wt.scale.y);
        const float sz = std::fabs(wt.scale.z);
        const float maxScale = std::max(sx, std::max(sy, sz));
        const float radius = sphere->radius * maxScale;

        const M::Vector3D oc = ray.origin - center;
        const float a = ray.dir.dot(ray.dir);
        const float b = 2.0f * oc.dot(ray.dir);
        const float c = oc.dot(oc) - radius * radius;

        const float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) {
            return false;
        }

        const float sqrtDisc = std::sqrt(disc);
        const float inv2a = 1.0f / (2.0f * a);

        float t0 = (-b - sqrtDisc) * inv2a;
        float t1 = (-b + sqrtDisc) * inv2a;
        if (t0 > t1) {
            std::swap(t0, t1);
        }

        float t = -1.0f;
        if (t0 >= 0.0f) {
            t = t0;
        } else if (t1 >= 0.0f) {
            t = t1;
        } else {
            return false;
        }

        if (t > maxDistance) {
            return false;
        }

        out.hit = true;
        out.t = t;
        out.fixture = const_cast<P::Fixture*>(&fixture);
        out.body = fixture.body;
        out.point = ray.pointAt(t);
        out.normal = (out.point - center).normalized();
        return true;
    }

    [[nodiscard]] bool raycastPlane(const M::Ray3D& ray,
                                    float maxDistance,
                                    const P::Fixture& fixture,
                                    P::RaycastHit& out) noexcept {
        const auto* planeCollider = dynamic_cast<const P::PlaneCollider*>(fixture.collider.get());
        if (!planeCollider) {
            return false;
        }

        M::Plane3D plane = planeCollider->plane;
        const M::Transform3D wt = fixture.worldTransform();

        const M::Point3D planePoint = transformPoint(wt, M::Point3D(
            plane.normal.x * (-plane.d),
            plane.normal.y * (-plane.d),
            plane.normal.z * (-plane.d)
        ));
        M::Vector3D planeNormal = transformVector(wt, plane.normal).normalized();

        const float denom = planeNormal.dot(ray.dir);
        if (std::fabs(denom) < M::EPS) {
            return false;
        }

        const float t = planeNormal.dot(planePoint - ray.origin) / denom;
        if (t < 0.0f || t > maxDistance) {
            return false;
        }

        out.hit = true;
        out.t = t;
        out.fixture = const_cast<P::Fixture*>(&fixture);
        out.body = fixture.body;
        out.point = ray.pointAt(t);
        out.normal = planeNormal;
        return true;
    }

    [[nodiscard]] bool raycastAABB(const M::Ray3D& ray,
                                   float maxDistance,
                                   const M::AABB3D& box,
                                   P::RaycastHit& out,
                                   P::Fixture* fixture,
                                   P::RigidBody* body) noexcept {
        float tmin = 0.0f;
        float tmax = maxDistance;
        M::Vector3D hitNormal(0.0f, 0.0f, 0.0f);

        const auto testAxis = [&](float origin, float dir, float minv, float maxv,
                                  const M::Vector3D& negNormal,
                                  const M::Vector3D& posNormal,
                                  float& ioTMin,
                                  float& ioTMax,
                                  M::Vector3D& ioNormal) -> bool {
            if (std::fabs(dir) < M::EPS) {
                return origin >= minv && origin <= maxv;
            }

            float invDir = 1.0f / dir;
            float t1 = (minv - origin) * invDir;
            float t2 = (maxv - origin) * invDir;

            M::Vector3D enterNormal = negNormal;
            if (t1 > t2) {
                std::swap(t1, t2);
                enterNormal = posNormal;
            }

            if (t1 > ioTMin) {
                ioTMin = t1;
                ioNormal = enterNormal;
            }
            ioTMax = std::min(ioTMax, t2);

            return ioTMin <= ioTMax;
        };

        if (!testAxis(ray.origin.x, ray.dir.x, box.min.x, box.max.x,
                      M::Vector3D(-1.0f, 0.0f, 0.0f), M::Vector3D(1.0f, 0.0f, 0.0f),
                      tmin, tmax, hitNormal)) {
            return false;
        }

        if (!testAxis(ray.origin.y, ray.dir.y, box.min.y, box.max.y,
                      M::Vector3D(0.0f, -1.0f, 0.0f), M::Vector3D(0.0f, 1.0f, 0.0f),
                      tmin, tmax, hitNormal)) {
            return false;
        }

        if (!testAxis(ray.origin.z, ray.dir.z, box.min.z, box.max.z,
                      M::Vector3D(0.0f, 0.0f, -1.0f), M::Vector3D(0.0f, 0.0f, 1.0f),
                      tmin, tmax, hitNormal)) {
            return false;
        }

        if (tmin < 0.0f || tmin > maxDistance) {
            return false;
        }

        out.hit = true;
        out.t = tmin;
        out.fixture = fixture;
        out.body = body;
        out.point = ray.pointAt(tmin);
        out.normal = hitNormal;
        return true;
    }

    [[nodiscard]] bool raycastBox(const M::Ray3D& ray,
                                  float maxDistance,
                                  const P::Fixture& fixture,
                                  P::RaycastHit& out) noexcept {
        const auto* box = dynamic_cast<const P::BoxCollider*>(fixture.collider.get());
        if (!box) {
            return false;
        }

        // Milestone-1 simplification: use world AABB of the box collider.
        return raycastAABB(ray,
                           maxDistance,
                           fixture.worldAABB(),
                           out,
                           const_cast<P::Fixture*>(&fixture),
                           fixture.body);
    }

    [[nodiscard]] bool intersectsSphereSphere(const P::Fixture& a,
                                              const P::Fixture& b) noexcept {
        const auto* sa = dynamic_cast<const P::SphereCollider*>(a.collider.get());
        const auto* sb = dynamic_cast<const P::SphereCollider*>(b.collider.get());
        if (!sa || !sb) {
            return false;
        }

        const M::Transform3D ta = a.worldTransform();
        const M::Transform3D tb = b.worldTransform();

        const float ra = sa->radius * std::max(std::fabs(ta.scale.x), std::max(std::fabs(ta.scale.y), std::fabs(ta.scale.z)));
        const float rb = sb->radius * std::max(std::fabs(tb.scale.x), std::max(std::fabs(tb.scale.y), std::fabs(tb.scale.z)));

        const float dist2 = ta.position.distanceSqrTo(tb.position);
        const float rsum = ra + rb;
        return dist2 <= rsum * rsum;
    }

    [[nodiscard]] bool intersectsSpherePlane(const P::Fixture& sphereFix,
                                             const P::Fixture& planeFix) noexcept {
        const auto* sphere = dynamic_cast<const P::SphereCollider*>(sphereFix.collider.get());
        const auto* planeCollider = dynamic_cast<const P::PlaneCollider*>(planeFix.collider.get());
        if (!sphere || !planeCollider) {
            return false;
        }

        const M::Transform3D ts = sphereFix.worldTransform();
        const M::Transform3D tp = planeFix.worldTransform();

        const float radius = sphere->radius * std::max(std::fabs(ts.scale.x), std::max(std::fabs(ts.scale.y), std::fabs(ts.scale.z)));

        const M::Point3D planePoint = transformPoint(tp, M::Point3D(
            planeCollider->plane.normal.x * (-planeCollider->plane.d),
            planeCollider->plane.normal.y * (-planeCollider->plane.d),
            planeCollider->plane.normal.z * (-planeCollider->plane.d)
        ));
        const M::Vector3D planeNormal = transformVector(tp, planeCollider->plane.normal).normalized();

        const float dist = planeNormal.dot(ts.position - planePoint);
        return std::fabs(dist) <= radius;
    }

    [[nodiscard]] bool intersectsBoxPlane(const P::Fixture& boxFix,
                                          const P::Fixture& planeFix) noexcept {
        const auto* planeCollider = dynamic_cast<const P::PlaneCollider*>(planeFix.collider.get());
        const auto* box = dynamic_cast<const P::BoxCollider*>(boxFix.collider.get());
        if (!planeCollider || !box) {
            return false;
        }

        const M::Transform3D tb = boxFix.worldTransform();
        const M::Transform3D tp = planeFix.worldTransform();

        const M::Vector3D he(
            std::fabs(box->halfExtents.x * tb.scale.x),
            std::fabs(box->halfExtents.y * tb.scale.y),
            std::fabs(box->halfExtents.z * tb.scale.z)
        );

        const M::Point3D planePoint = transformPoint(tp, M::Point3D(
            planeCollider->plane.normal.x * (-planeCollider->plane.d),
            planeCollider->plane.normal.y * (-planeCollider->plane.d),
            planeCollider->plane.normal.z * (-planeCollider->plane.d)
        ));
        const M::Vector3D planeNormal = transformVector(tp, planeCollider->plane.normal).normalized();

        // AABB-style projected radius against the plane normal.
        const float projectedRadius =
            std::fabs(planeNormal.x) * he.x +
            std::fabs(planeNormal.y) * he.y +
            std::fabs(planeNormal.z) * he.z;

        const float dist = planeNormal.dot(tb.position - planePoint);
        return std::fabs(dist) <= projectedRadius;
    }
}

namespace P {

    RaycastHit raycast(const World& world,
                       const M::Ray3D& ray,
                       float maxDistance) noexcept {
        RaycastHit best{};
        best.hit = false;
        best.t = maxDistance;

        for (const auto& fixturePtr : world.fixtures) {
            if (!fixturePtr) {
                continue;
            }

            const Fixture& fixture = *fixturePtr;
            if (!fixture.enabled || !fixture.valid()) {
                continue;
            }

            RaycastHit current{};
            bool hit = false;

            switch (fixture.collider->type()) {
                case ShapeType::Sphere:
                    hit = raycastSphere(ray, maxDistance, fixture, current);
                    break;
                case ShapeType::Box:
                    hit = raycastBox(ray, maxDistance, fixture, current);
                    break;
                case ShapeType::Plane:
                    hit = raycastPlane(ray, maxDistance, fixture, current);
                    break;
            }

            if (hit && (!best.hit || current.t < best.t)) {
                best = current;
            }
        }

        return best;
    }

    bool intersects(const Fixture& a,
                    const Fixture& b) noexcept {
        if (!a.valid() || !b.valid()) {
            return false;
        }

        const ShapeType ta = a.collider->type();
        const ShapeType tb = b.collider->type();

        if (ta == ShapeType::Sphere && tb == ShapeType::Sphere) {
            return intersectsSphereSphere(a, b);
        }

        if (ta == ShapeType::Sphere && tb == ShapeType::Plane) {
            return intersectsSpherePlane(a, b);
        }

        if (ta == ShapeType::Plane && tb == ShapeType::Sphere) {
            return intersectsSpherePlane(b, a);
        }

        if (ta == ShapeType::Box && tb == ShapeType::Plane) {
            return intersectsBoxPlane(a, b);
        }

        if (ta == ShapeType::Plane && tb == ShapeType::Box) {
            return intersectsBoxPlane(b, a);
        }

        // Milestone-1 fallback: AABB overlap.
        return M::intersects(a.worldAABB(), b.worldAABB());
    }

} // namespace P