#include "PBroadPhase.hpp"

#include "PFixture.hpp"

namespace P {

    bool BroadPhaseProxy::valid() const noexcept {
        return fixture != nullptr && aabb.isValid();
    }

    void BruteForceBroadPhase::clear() noexcept {
        proxies.clear();
    }

    void BruteForceBroadPhase::build(const std::vector<Fixture*>& fixtures) {
        proxies.clear();
        proxies.reserve(fixtures.size());

        for (Fixture* fixture : fixtures) {
            if (fixture == nullptr) {
                continue;
            }
            if (!fixture->enabled) {
                continue;
            }
            if (!fixture->valid()) {
                continue;
            }

            BroadPhaseProxy proxy;
            proxy.fixture = fixture;
            proxy.aabb = fixture->worldAABB();

            if (!proxy.valid()) {
                continue;
            }

            proxies.push_back(proxy);
        }
    }

    std::vector<ContactPair> BruteForceBroadPhase::computePairs() const {
        std::vector<ContactPair> pairs;

        const std::size_t n = proxies.size();
        if (n < 2) {
            return pairs;
        }

        pairs.reserve((n * (n - 1)) / 2);

        for (std::size_t i = 0; i < n; ++i) {
            const BroadPhaseProxy& a = proxies[i];
            if (!a.valid()) {
                continue;
            }

            for (std::size_t j = i + 1; j < n; ++j) {
                const BroadPhaseProxy& b = proxies[j];
                if (!b.valid()) {
                    continue;
                }

                if (a.fixture == b.fixture) {
                    continue;
                }

                if (!a.fixture->canCollideWith(*b.fixture)) {
                    continue;
                }

                if (!M::intersects(a.aabb, b.aabb)) {
                    continue;
                }

                ContactPair pair;
                pair.a = a.fixture;
                pair.b = b.fixture;

                if (pair.valid()) {
                    pairs.push_back(pair);
                }
            }
        }

        return pairs;
    }

} // namespace P