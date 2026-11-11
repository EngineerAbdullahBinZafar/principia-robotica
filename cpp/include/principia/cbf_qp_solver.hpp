/*
 * cbf_qp_solver.hpp
 * Real-Time Zero-Allocation C++17 Control Barrier Function (CBF-QP) Filter
 *
 * Implements closed-form single-constraint KKT projection and multi-constraint
 * Dykstra/active-set projection with box actuator constraints for differential
 * drive / unicycle mobile robots.
 *
 * Author: Engineer Abdullah Bin Zafar
 * License: MIT
 */

#ifndef PRINCIPIA_CBF_QP_SOLVER_HPP_
#define PRINCIPIA_CBF_QP_SOLVER_HPP_

#include <array>
#include <cstddef>

namespace principia {

constexpr std::size_t MAX_OBSTACLES = 16U;

struct UnicycleState {
    double px;
    double py;
    double theta;
};

struct UnicycleControl {
    double v;
    double omega;
};

struct CircularObstacle {
    double cx;
    double cy;
    double radius;
};

struct ActuatorLimits {
    double v_min{-0.5};
    double v_max{1.5};
    double omega_min{-2.5};
    double omega_max{2.5};
};

struct SolverResult {
    UnicycleControl u_safe;
    double          min_barrier_h;
    bool            intervention_active;
    bool            feasible;
    double          kkt_residual;
};

class UnicycleCBFFilter {
public:
    explicit UnicycleCBFFilter(double gamma = 4.0,
                               double lookahead_eps = 0.08,
                               ActuatorLimits limits = ActuatorLimits{}) noexcept;

    /**
     * @brief Evaluate safety barrier function h(x) = ||p - c||^2 - (r + margin)^2
     * incorporating small lookahead shift along heading so omega has direct Lie
     * derivative authority L_g h != 0 even when pointing directly toward obstacle.
     */
    double evaluate_barrier(const UnicycleState& state,
                            const CircularObstacle& obs) const noexcept;

    /**
     * @brief Project nominal AI control command u_nom onto forward-invariant safe set.
     * Zero heap allocation; suitable for deterministic >100 kHz RTOS / C++ loops.
     */
    SolverResult filter(const UnicycleState& state,
                        const UnicycleControl& u_nom,
                        const CircularObstacle* obstacles,
                        std::size_t num_obstacles) const noexcept;

    static UnicycleState step_unicycle(const UnicycleState& state,
                                       const UnicycleControl& u,
                                       double dt) noexcept;

private:
    double gamma_;
    double lookahead_eps_;
    ActuatorLimits limits_;

    UnicycleControl clamp_box(UnicycleControl u) const noexcept;
};

}  // namespace principia

#endif  // PRINCIPIA_CBF_QP_SOLVER_HPP_
