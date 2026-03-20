#ifndef P_WORLD_HPP
#define P_WORLD_HPP

#include <memory>
#include <vector>
#include "M.hpp"
#include "PBody.hpp"
#include "PFixture.hpp"
#include "PBroadPhase.hpp"
#include "PSolver.hpp"
#include "PContact.hpp"

namespace P {

    float restingLinearThreshold = 0.35f;
    float restingAngularThreshold = 0.9f;

    float settleAssistTorque = 18.0f;
    float settleSnapDot = 0.9965f;

    struct WorldSettings {
        M::Vector3D gravity {0.0f, -9.81f, 0.0f};

        bool enableSleeping = true;
        bool enableBroadPhase = true;

        float maxLinearSpeed = 1000.0f;
        float maxAngularSpeed = 1000.0f;

        SolverSettings solver;
    };

    class World {
    public:
        WorldSettings settings;

        std::vector<std::unique_ptr<RigidBody>> bodies;
        std::vector<std::unique_ptr<Fixture>> fixtures;

        std::unique_ptr<BroadPhase> broadPhase;
        std::unique_ptr<ContactSolver> solver;

        std::vector<ContactPair> broadPhasePairs;
        std::vector<ContactManifold> manifolds;
        std::vector<ContactConstraint> constraints;

        World();
        explicit World(const WorldSettings& ws);

        [[nodiscard]] RigidBody* createBody();
        [[nodiscard]] Fixture* createFixture(RigidBody* body, ColliderPtr collider);

        void clear() noexcept;

        void step(float dt);

        void integrateForces(float dt) noexcept;
        void integrateVelocities(float dt) noexcept;

        void buildBroadPhase();
        void buildManifolds();
        void buildConstraints();

        void solveConstraints(float dt);

        void clearAccumulators() noexcept;
        void stabilizeRestingBoxes(float dt) noexcept
    };

    [[nodiscard]] ContactManifold collide(
        Fixture& a,
        Fixture& b) noexcept;

    [[nodiscard]] ContactManifold collideSphereSphere(
        Fixture& a,
        Fixture& b) noexcept;

    [[nodiscard]] ContactManifold collideSpherePlane(
        Fixture& sphereFixture,
        Fixture& planeFixture) noexcept;

    [[nodiscard]] ContactManifold collideBoxPlane(
        Fixture& boxFixture,
        Fixture& planeFixture) noexcept;

    [[nodiscard]] ContactManifold collideBoxBox(
        Fixture& a,
        Fixture& b) noexcept;
        
    void stabilizeRestingBoxes(float dt) noexcept;
} // namespace P

#endif