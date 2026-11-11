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

static void test_multi_obstacle_box_bounds()
{
    UnicycleCBFFilter cbf(5.0, 0.10);
    UnicycleState x{0.0, 0.0, 0.0};
    CircularObstacle obs[2] = {
        {1.0, 0.45, 0.4},
        {1.0, -0.45, 0.4}
    };
    UnicycleControl u_extreme{5.0, 0.0}; /* Exceeds actuator v_max=1.5 */

    auto res = cbf.filter(x, u_extreme, obs, 2U);
    assert(res.feasible);
    assert(res.u_safe.v <= 1.5 + 1e-9);
    assert(res.u_safe.v >= -0.5 - 1e-9);
    assert(std::fabs(res.u_safe.omega) <= 2.5 + 1e-9);
    std::cout << "  [PASS] test_multi_obstacle_box_bounds\n";
}

int main()
{
    std::cout << "Running Principia Robotica C++17 CBF-QP Safety Suite...\n";
    test_nominal_passthrough_when_safe();
    test_head_on_collision_deflection_invariance();
    test_multi_obstacle_box_bounds();
    std::cout << "ALL C++17 CBF-QP TESTS PASSED.\n";
    return 0;
}
