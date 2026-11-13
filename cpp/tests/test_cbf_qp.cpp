/*
 * test_cbf_qp.cpp
 * C++17 Verification Suite for Closed-Loop Forward Invariance (Nagumo Theorem)
 * Compiled with -std=c++17 -O3 -Wall -Wextra -Werror -fsanitize=address,undefined
 */

#include "principia/cbf_qp_solver.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace principia;

static void test_nominal_passthrough_when_safe()
{
    UnicycleCBFFilter cbf(4.0, 0.08);
    UnicycleState x{0.0, 0.0, 0.0};
    CircularObstacle obs{5.0, 5.0, 0.5};
    UnicycleControl u_nom{1.0, 0.1};

    auto res = cbf.filter(x, u_nom, &obs, 1U);
    assert(res.feasible);
    assert(!res.intervention_active);
    assert(std::fabs(res.u_safe.v - 1.0) < 1e-9);
    assert(std::fabs(res.u_safe.omega - 0.1) < 1e-9);
    std::cout << "  [PASS] test_nominal_passthrough_when_safe\n";
}

static void test_head_on_collision_deflection_invariance()
{
    UnicycleCBFFilter cbf(4.0, 0.10);
    /* Robot heading straight (+x) at full speed toward obstacle at (1.5, 0.05) */
    UnicycleState x{0.0, 0.0, 0.0};
    CircularObstacle obs{1.5, 0.05, 0.6};
    UnicycleControl u_malicious{1.5, 0.0}; /* Unsafe AI full forward command */

    double min_observed_h = 1e9;
    bool intervened_at_least_once = false;

    for (int step = 0; step < 300; ++step) {
        auto res = cbf.filter(x, u_malicious, &obs, 1U);
        assert(res.feasible);
        if (res.intervention_active) {
            intervened_at_least_once = true;
        }
        min_observed_h = std::min(min_observed_h, res.min_barrier_h);
        assert(res.min_barrier_h >= -1e-2);
        x = UnicycleCBFFilter::step_unicycle(x, res.u_safe, 0.01);
    }

    assert(intervened_at_least_once);
    assert(min_observed_h >= -1e-2);
    std::cout << "  [PASS] test_head_on_collision_deflection_invariance (min_h="
              << min_observed_h << ")\n";
}

static void assert_all_halfspaces_satisfied(const UnicycleState& x,
                                          double gamma,
                                          double lookahead_eps,
                                          const CircularObstacle* obs,
                                          std::size_t count,
                                          const UnicycleControl& u,
                                          double tol = 1e-6)
{
    const double ct = std::cos(x.theta);
    const double st = std::sin(x.theta);
    const double lx = x.px + lookahead_eps * ct;
    const double ly = x.py + lookahead_eps * st;
    for (std::size_t i = 0U; i < count; ++i) {
        const double dx = lx - obs[i].cx;
        const double dy = ly - obs[i].cy;
        const double h = (dx * dx + dy * dy) - (obs[i].radius * obs[i].radius);
        const double a0 = 2.0 * (dx * ct + dy * st);
        const double a1 = 2.0 * lookahead_eps * (-dx * st + dy * ct);
        const double b = -gamma * h;
        const double lhs = a0 * u.v + a1 * u.omega;
        assert(lhs >= b - tol);
    }
}

static void test_multi_obstacle_box_bounds()
{
    const double gamma = 5.0;
    const double eps = 0.10;
    UnicycleCBFFilter cbf(gamma, eps);
    UnicycleState x{0.0, 0.0, 0.0};
    CircularObstacle obs[2] = {
        {1.0, 0.45, 0.4},
        {1.0, -0.45, 0.4}
    };
    UnicycleControl u_extreme{5.0, 0.0}; /* Exceeds actuator v_max=1.5 */

    auto res = cbf.filter(x, u_extreme, obs, 2U);
    assert(res.feasible);
    assert(res.intervention_active);
    assert(res.u_safe.v <= 1.5 + 1e-9);
    assert(res.u_safe.v >= -0.5 - 1e-9);
    assert(std::fabs(res.u_safe.omega) <= 2.5 + 1e-9);
    assert_all_halfspaces_satisfied(x, gamma, eps, obs, 2U, res.u_safe);
    std::cout << "  [PASS] test_multi_obstacle_box_bounds\n";
}

static void test_multi_obstacle_conflict_and_objective_preservation()
{
    const double gamma = 1.8;
    const double eps = 0.25;
    UnicycleCBFFilter cbf(gamma, eps);
    UnicycleState x{0.35, 0.02, 0.0};
    CircularObstacle obs[2] = {
        {1.0, 0.45, 0.65},
        {1.0, -0.45, 0.65}
    };
    UnicycleControl u_nom{1.5, -2.0};

    auto res = cbf.filter(x, u_nom, obs, 2U);
    assert(res.feasible);
    assert(res.u_safe.v >= -0.5 - 1e-9 && res.u_safe.v <= 1.5 + 1e-9);
    assert(std::fabs(res.u_safe.omega) <= 2.5 + 1e-9);
    assert_all_halfspaces_satisfied(x, gamma, eps, obs, 2U, res.u_safe);

    /* Objective preservation check when nominal exceeds box bounds:
     * Pre-clamping (5.0, 3.0) -> (1.5, 2.5) before projection yields (-0.3, 1.6)
     * with ||u - u_nom||^2 = 30.05, whereas the true QP optimum is (0.2, 0.6)
     * with ||u - u_nom||^2 = 28.80.
     */
    const double g2 = 2.0;
    const double eps2 = 0.5;
    UnicycleCBFFilter cbf_obj(g2, eps2);
    UnicycleState x_obj{0.0, 0.0, 0.0};
    CircularObstacle obs_obj{1.0, 0.5, 0.5}; /* lx=0.5, dx=-0.5, dy=-0.5 => a0=-1, a1=-0.5, h=0.25, b=-0.5 */
    UnicycleControl u_out{5.0, 3.0};
    auto res_obj = cbf_obj.filter(x_obj, u_out, &obs_obj, 1U);
    assert(res_obj.feasible);
    assert_all_halfspaces_satisfied(x_obj, g2, eps2, &obs_obj, 1U, res_obj.u_safe);
    const double d_opt_sq =
        (res_obj.u_safe.v - u_out.v) * (res_obj.u_safe.v - u_out.v) +
        (res_obj.u_safe.omega - u_out.omega) * (res_obj.u_safe.omega - u_out.omega);
    assert(d_opt_sq <= 28.80 + 1e-6);
    assert(std::fabs(res_obj.u_safe.v - 0.2) < 1e-6);
    assert(std::fabs(res_obj.u_safe.omega - 0.6) < 1e-6);
    std::cout << "  [PASS] test_multi_obstacle_conflict_and_objective_preservation\n";
}

static void test_infeasible_opposing_constraints_slack()
{
    UnicycleCBFFilter cbf(4.0, 0.10);
    UnicycleState x{0.0, 0.0, 0.0};
    /* Opposing obstacles overlapping robot position -> h < 0 in opposite directions */
    CircularObstacle obs[2] = {
        {0.5, 0.0, 0.65},
        {-0.5, 0.0, 0.65}
    };
    UnicycleControl u_nom{1.0, 0.5};

    auto res = cbf.filter(x, u_nom, obs, 2U);
    assert(!res.feasible);
    assert(res.intervention_active);
    assert(res.kkt_residual > 0.0);
    assert(res.u_safe.v >= -0.5 - 1e-9 && res.u_safe.v <= 1.5 + 1e-9);
    assert(std::fabs(res.u_safe.omega) <= 2.5 + 1e-9);
    std::cout << "  [PASS] test_infeasible_opposing_constraints_slack\n";
}

int main()
{
    std::cout << "Running Principia Robotica C++17 CBF-QP Safety Suite...\n";
    test_nominal_passthrough_when_safe();
    test_head_on_collision_deflection_invariance();
    test_multi_obstacle_box_bounds();
    test_multi_obstacle_conflict_and_objective_preservation();
    test_infeasible_opposing_constraints_slack();
    std::cout << "ALL C++17 CBF-QP TESTS PASSED.\n";
    return 0;
}
