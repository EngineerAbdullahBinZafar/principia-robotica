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

namespace {

struct HalfSpace {
    double a0;
    double a1;
    double b;
    double h;
};

struct NormHalfSpace {
    double n0;
    double n1;
    double d;
};

bool solve_2d_with_shift(const UnicycleControl& u_nom,
                         const ActuatorLimits& limits,
                         const std::array<HalfSpace, MAX_OBSTACLES>& hs,
                         std::size_t count,
                         double delta,
                         UnicycleControl& out_u) noexcept
{
    std::array<NormHalfSpace, MAX_OBSTACLES + 4U> active_hps{};
    std::size_t active_count = 0U;

    active_hps[active_count++] = NormHalfSpace{1.0, 0.0, limits.v_min};
    active_hps[active_count++] = NormHalfSpace{-1.0, 0.0, -limits.v_max};
    active_hps[active_count++] = NormHalfSpace{0.0, 1.0, limits.omega_min};
    active_hps[active_count++] = NormHalfSpace{0.0, -1.0, -limits.omega_max};

    std::array<NormHalfSpace, MAX_OBSTACLES> normed_cbfs{};
    std::size_t normed_count = 0U;

    for (std::size_t i = 0U; i < count; ++i) {
        const double b_shift = hs[i].b - delta;
        const double norm = std::hypot(hs[i].a0, hs[i].a1);
        if (norm <= 1e-12) {
            if (b_shift > 1e-9) {
                return false;
            }
        } else {
            normed_cbfs[normed_count++] = NormHalfSpace{
                hs[i].a0 / norm,
                hs[i].a1 / norm,
                b_shift / norm
            };
        }
    }

    UnicycleControl u{
        std::max(limits.v_min, std::min(limits.v_max, u_nom.v)),
        std::max(limits.omega_min, std::min(limits.omega_max, u_nom.omega))
    };

    for (std::size_t i = 0U; i < normed_count; ++i) {
        const NormHalfSpace& cbf = normed_cbfs[i];
        if (cbf.n0 * u.v + cbf.n1 * u.omega < cbf.d - 1e-12) {
            const double s_nom = cbf.n0 * u_nom.v + cbf.n1 * u_nom.omega;
            const double pv = u_nom.v + (cbf.d - s_nom) * cbf.n0;
            const double pw = u_nom.omega + (cbf.d - s_nom) * cbf.n1;
            const double dv = -cbf.n1;
            const double dw = cbf.n0;

            bool has_t_min = false;
            bool has_t_max = false;
            double t_min = 0.0;
            double t_max = 0.0;

            for (std::size_t j = 0U; j < active_count; ++j) {
                const NormHalfSpace& prior = active_hps[j];
                const double alpha = prior.n0 * dv + prior.n1 * dw;
                const double beta = prior.d - (prior.n0 * pv + prior.n1 * pw);
                if (alpha > 1e-11) {
                    const double val = beta / alpha;
                    if (!has_t_min || val > t_min) {
                        t_min = val;
                        has_t_min = true;
                    }
                } else if (alpha < -1e-11) {
                    const double val = beta / alpha;
                    if (!has_t_max || val < t_max) {
                        t_max = val;
                        has_t_max = true;
                    }
                } else {
                    if (beta > 1e-9) {
                        return false;
                    }
                }
            }

            if (has_t_min && has_t_max && t_min > t_max + 1e-9) {
                return false;
            }

            double t_opt = 0.0;
            if (has_t_min && has_t_max && t_min > t_max) {
                t_opt = 0.5 * (t_min + t_max);
            } else {
                if (has_t_min && t_opt < t_min) {
                    t_opt = t_min;
                }
                if (has_t_max && t_opt > t_max) {
                    t_opt = t_max;
                }
            }

            u.v = pv + t_opt * dv;
            u.omega = pw + t_opt * dw;
        }
        active_hps[active_count++] = cbf;
    }

    u.v = std::max(limits.v_min, std::min(limits.v_max, u.v));
    u.omega = std::max(limits.omega_min, std::min(limits.omega_max, u.omega));

    for (std::size_t i = 0U; i < count; ++i) {
        const double lhs = hs[i].a0 * u.v + hs[i].a1 * u.omega;
        if (lhs < (hs[i].b - delta) - 1e-6) {
            return false;
        }
    }

    out_u = u;
    return true;
}

}  // namespace

SolverResult UnicycleCBFFilter::filter(const UnicycleState& state,
                                       const UnicycleControl& u_nom,
                                       const CircularObstacle* obstacles,
                                       std::size_t num_obstacles) const noexcept
{
    double min_h = std::numeric_limits<double>::infinity();

    if (limits_.v_min > limits_.v_max || limits_.omega_min > limits_.omega_max) {
        const UnicycleControl mid{
            0.5 * (limits_.v_min + limits_.v_max),
            0.5 * (limits_.omega_min + limits_.omega_max)
        };
        return SolverResult{mid, min_h, true, false, std::numeric_limits<double>::infinity()};
    }

    const UnicycleControl u_box = clamp_box(u_nom);
    const bool box_clamped =
        (std::fabs(u_box.v - u_nom.v) > 1e-9 || std::fabs(u_box.omega - u_nom.omega) > 1e-9);

    if (obstacles == nullptr || num_obstacles == 0U) {
        return SolverResult{u_box, min_h, box_clamped, true, 0.0};
    }

    const double ct = std::cos(state.theta);
    const double st = std::sin(state.theta);
    const double lx = state.px + lookahead_eps_ * ct;
    const double ly = state.py + lookahead_eps_ * st;

    std::array<HalfSpace, MAX_OBSTACLES> hs{};
    std::size_t count = 0U;

    for (std::size_t i = 0U; i < num_obstacles; ++i) {
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
        const HalfSpace cand{a0, a1, b, h};

        if (count < MAX_OBSTACLES) {
            hs[count++] = cand;
        } else {
            std::size_t max_h_idx = 0U;
            for (std::size_t k = 1U; k < MAX_OBSTACLES; ++k) {
                if (hs[k].h > hs[max_h_idx].h) {
                    max_h_idx = k;
                }
            }
            if (cand.h < hs[max_h_idx].h) {
                hs[max_h_idx] = cand;
            }
        }
    }

    /* Fast path: if u_box satisfies all CBF constraints, it is the exact global QP optimum */
    bool box_safe = true;
    for (std::size_t i = 0U; i < count; ++i) {
        const double lhs = hs[i].a0 * u_box.v + hs[i].a1 * u_box.omega;
        if (lhs < hs[i].b - 1e-12) {
            box_safe = false;
            break;
        }
    }
    if (box_safe) {
        return SolverResult{u_box, min_h, box_clamped, true, 0.0};
    }

    /* Exact 2D QP projection starting from un-clamped u_nom */
    UnicycleControl u_sol = u_box;
    if (solve_2d_with_shift(u_nom, limits_, hs, count, 0.0, u_sol)) {
        double max_viol = 0.0;
        for (std::size_t i = 0U; i < count; ++i) {
            const double lhs = hs[i].a0 * u_sol.v + hs[i].a1 * u_sol.omega;
            max_viol = std::max(max_viol, hs[i].b - lhs);
        }
        const bool modified =
            (std::fabs(u_sol.v - u_nom.v) > 1e-9 || std::fabs(u_sol.omega - u_nom.omega) > 1e-9);
        return SolverResult{u_sol, min_h, modified, true, max_viol};
    }

    /* Infeasible intersection: binary search minimum worst-case CBF violation delta >= 0 inside box */
    double d_hi = 0.0;
    for (std::size_t i = 0U; i < count; ++i) {
        const double lhs = hs[i].a0 * u_box.v + hs[i].a1 * u_box.omega;
        d_hi = std::max(d_hi, hs[i].b - lhs);
    }
    d_hi = d_hi * 1.01 + 1e-6;
    double d_lo = 0.0;
    UnicycleControl best_u = u_box;

    for (int iter = 0; iter < 42; ++iter) {
        const double mid = 0.5 * (d_lo + d_hi);
        UnicycleControl u_mid = u_box;
        if (solve_2d_with_shift(u_nom, limits_, hs, count, mid, u_mid)) {
            d_hi = mid;
            best_u = u_mid;
        } else {
            d_lo = mid;
        }
    }

    double final_viol = 0.0;
    for (std::size_t i = 0U; i < count; ++i) {
        const double lhs = hs[i].a0 * best_u.v + hs[i].a1 * best_u.omega;
        final_viol = std::max(final_viol, hs[i].b - lhs);
    }
    return SolverResult{best_u, min_h, true, false, final_viol};
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
