#ifndef P_SOLVER_HPP
#define P_SOLVER_HPP

#include <vector>
#include "PContact.hpp"

namespace P {

    class World;

    struct SolverSettings {
        int velocityIterations = 8;
        int positionIterations = 3;

        float baumgarte = 0.2f;
        float penetrationSlop = 0.005f;

        bool enableFriction = true;
        bool enableRestitution = true;
    };

    class ContactSolver {
    public:
        virtual ~ContactSolver() = default;

        virtual void solve(
            World& world,
            std::vector<ContactConstraint>& constraints,
            float dt,
            const SolverSettings& settings) = 0;
    };

    class SequentialImpulseSolver final : public ContactSolver {
    public:
        void solve(
            World& world,
            std::vector<ContactConstraint>& constraints,
            float dt,
            const SolverSettings& settings) override;
    };

} // namespace P

#endif