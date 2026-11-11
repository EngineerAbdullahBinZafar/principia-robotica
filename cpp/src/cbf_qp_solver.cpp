/*
 * cbf_qp_solver.cpp
 * Deterministic C++17 Minimum-Intervention CBF-QP Safety Filter Implementation
 *
 * Author: Engineer Abdullah Bin Zafar
 * License: MIT
 */

#include "principia/cbf_qp_solver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace principia {

UnicycleCBFFilter::UnicycleCBFFilter(double gamma,
                                     double lookahead_eps,
                                     ActuatorLimits limits) noexcept
    : gamma_(gamma)
    , lookahead_eps_(lookahead_eps)
    , limits_(limits)
{
}

UnicycleControl UnicycleCBFFilter::clamp_box(UnicycleControl u) const noexcept
{
    u.v = std::max(limits_.v_min, std::min(limits_.v_max, u.v));
    u.omega = std::max(limits_.omega_min, std::min(limits_.omega_max, u.omega));
    return u;
}

double UnicycleCBFFilter::evaluate_barrier(const UnicycleState& state,
                                           const CircularObstacle& obs) const noexcept
{
    const double c = std::cos(state.theta);
    const double s = std::sin(state.theta);
    const double lx = state.px + lookahead_eps_ * c;
    const double ly = state.py + lookahead_eps_ * s;
    const double dx = lx - obs.cx;
    const double dy = ly - obs.cy;
    return (dx * dx + dy * dy) - (obs.radius * obs.radius);
}

SolverResult UnicycleCBFFilter::filter(const UnicycleState& state,
                                       const UnicycleControl& u_nom,
                                       const CircularObstacle* obstacles,
                                       std::size_t num_obstacles) const noexcept
{
    UnicycleControl u = clamp_box(u_nom);
    double min_h = std::numeric_limits<double>::infinity();

    if (obstacles == nullptr || num_obstacles == 0U) {
        return SolverResult{u, min_h, false, true, 0.0};
    }

    const std::size_t count = std::min(num_obstacles, MAX_OBSTACLES);
    const double ct = std::cos(state.theta);
    const double st = std::sin(state.theta);
    const double lx = state.px + lookahead_eps_ * ct;
    const double ly = state.py + lookahead_eps_ * st;

    struct HalfSpace {
        double a0;
        double a1;
        double b;
        double h;
    };

    std::array<HalfSpace, MAX_OBSTACLES> hs{};
    for (std::size_t i = 0U; i < count; ++i) {
        const double dx = lx - obstacles[i].cx;
        const double dy = ly - obstacles[i].cy;
        const double h = (dx * dx + dy * dy) - (obstacles[i].radius * obstacles[i].radius);
        min_h = std::min(min_h, h);

        /* a^T u >= b where:
         * a0 = d(h)/dx * cos(th) + d(h)/dy * sin(th)
         * a1 = lookahead_eps * (-d(h)/dx * sin(th) + d(h)/dy * cos(th))
         * b  = -gamma * h
         */
        const double a0 = 2.0 * (dx * ct + dy * st);
        const double a1 = 2.0 * lookahead_eps_ * (-dx * st + dy * ct);
        const double b = -gamma_ * h;
        hs[i] = HalfSpace{a0, a1, b, h};
    }

    bool intervened = false;
    /* Single obstacle closed-form KKT when possible, then Dykstra projection for multi-obstacle + box */
    std::array<UnicycleControl, MAX_OBSTACLES + 1U> dual_inc{};
    constexpr int MAX_ITERS = 32;

    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        double max_violation = 0.0;

        for (std::size_t i = 0U; i < count; ++i) {
            const double norm_sq = hs[i].a0 * hs[i].a0 + hs[i].a1 * hs[i].a1;
            if (norm_sq < 1e-12) {
                continue;
            }
            const UnicycleControl p{u.v + dual_inc[i].v, u.omega + dual_inc[i].omega};
            const double lhs = hs[i].a0 * p.v + hs[i].a1 * p.omega;
            const double viol = hs[i].b - lhs;
            UnicycleControl projected = p;
            if (viol > 0.0) {
                const double step = viol / norm_sq;
                projected.v += step * hs[i].a0;
                projected.omega += step * hs[i].a1;
                intervened = true;
            }
            dual_inc[i].v = p.v - projected.v;
            dual_inc[i].omega = p.omega - projected.omega;
            u = projected;
        }

        /* Project onto actuator box constraints */
        const UnicycleControl p_box{u.v + dual_inc[count].v, u.omega + dual_inc[count].omega};
        const UnicycleControl boxed = clamp_box(p_box);
        dual_inc[count].v = p_box.v - boxed.v;
        dual_inc[count].omega = p_box.omega - boxed.omega;
        u = boxed;

        for (std::size_t i = 0U; i < count; ++i) {
            const double lhs = hs[i].a0 * u.v + hs[i].a1 * u.omega;
            max_violation = std::max(max_violation, hs[i].b - lhs);
        }
        if (max_violation <= 1e-6) {
            return SolverResult{u, min_h, intervened, true, max_violation};
        }
    }

    double final_viol = 0.0;
    for (std::size_t i = 0U; i < count; ++i) {
        const double lhs = hs[i].a0 * u.v + hs[i].a1 * u.omega;
        final_viol = std::max(final_viol, hs[i].b - lhs);
    }

    return SolverResult{u, min_h, intervened, final_viol <= 1e-3, std::max(0.0, final_viol)};
}

UnicycleState UnicycleCBFFilter::step_unicycle(const UnicycleState& state,
                                               const UnicycleControl& u,
                                               double dt) noexcept
{
    return UnicycleState{
        state.px + u.v * std::cos(state.theta) * dt,
        state.py + u.v * std::sin(state.theta) * dt,
        state.theta + u.omega * dt
    };
}

}  // namespace principia
