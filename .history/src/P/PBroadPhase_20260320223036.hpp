#ifndef P_BROAD_PHASE_HPP
#define P_BROAD_PHASE_HPP

#include <vector>
#include "M.hpp"
#include "PContact.hpp"

namespace P {
    [[nodiscard]] M::Point3D transformPointOnly(const M::Transform3D& t, const M::Point3D& p) noexcept {
    const M::Vector3D scaled(
        p.x * t.scale.x,
        p.y * t.scale.y,
        p.z * t.scale.z
    );
    return t.position + t.rotation.rotatedVector(scaled);
}

    class Fixture;

    struct BroadPhaseProxy {
        Fixture* fixture = nullptr;
        M::AABB3D aabb;

        [[nodiscard]] bool valid() const noexcept;
    };

    class BroadPhase {
    public:
        virtual ~BroadPhase() = default;

        virtual void clear() noexcept = 0;
        virtual void build(const std::vector<Fixture*>& fixtures) = 0;

        [[nodiscard]] virtual std::vector<ContactPair> computePairs() const = 0;
    };

    class BruteForceBroadPhase final : public BroadPhase {
    public:
        std::vector<BroadPhaseProxy> proxies;

        void clear() noexcept override;
        void build(const std::vector<Fixture*>& fixtures) override;

        [[nodiscard]] std::vector<ContactPair> computePairs() const override;
    };

} // namespace P

#endif