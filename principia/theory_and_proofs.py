"""
Principia Robotica — THEORY & PROOFS (Mathematical Foundations)

This document contains the complete mathematical derivations
for all algorithms implemented in this package.
"""

THEORY = """
╔══════════════════════════════════════════════════════════════════════════════╗
║         PRINCIPIA ROBOTICA — THEORY AND MATHEMATICAL PROOFS                 ║
║         Author: Abdullah Bin Zafar | UET Lahore | 2026                     ║
╚══════════════════════════════════════════════════════════════════════════════╝

─────────────────────────────────────────────────────────────────────────────
§1  CONTROL BARRIER FUNCTIONS (CBF) — FORMAL DEFINITION
─────────────────────────────────────────────────────────────────────────────

DEFINITION 1 (Control Barrier Function):
    Consider a control-affine dynamical system:
        ẋ = f(x) + g(x)u,   x ∈ ℝⁿ, u ∈ U ⊆ ℝᵐ

    Let h: ℝⁿ → ℝ be a continuously differentiable function defining:
        C = {x ∈ ℝⁿ : h(x) ≥ 0}  [safe set]

    h is a Valid Control Barrier Function (CBF) for (f, g) on C if:
        ∃ α ∈ K∞ such that:
        sup_{u∈U} [Lf h(x) + Lg h(x) u] ≥ -α(h(x))   ∀x ∈ C

    where:
        Lf h(x) = ∂h/∂x · f(x)       [Lie derivative along drift]
        Lg h(x) = ∂h/∂x · g(x)       [Lie derivative along control]
        α: ℝ → ℝ is class-K (strictly increasing, α(0)=0)

THEOREM 1 (Forward Invariance via CBF — Ames et al., 2014):
    If h is a valid CBF for (f, g) and u(x) satisfies:
        Lf h(x) + Lg h(x) u(x) ≥ -α(h(x))   ∀x ∈ C

    Then C is forward invariant:
        x(0) ∈ C  ⟹  x(t) ∈ C   ∀t ≥ 0

PROOF:
    Define V(t) = h(x(t)) along closed-loop trajectories.
    V̇(t) = Lf h(x) + Lg h(x) u(x)
           ≥ -α(h(x))
           = -α(V(t))

    By comparison lemma, V(t) ≥ β(V(0), t) > 0 for V(0) > 0,
    where β is a class-KL function. □

─────────────────────────────────────────────────────────────────────────────
§2  CBF-QP SAFETY FILTER — OPTIMALITY PROOF
─────────────────────────────────────────────────────────────────────────────

DEFINITION 2 (CBF-QP Safety Filter):
    Given desired control u_AI ∈ ℝᵐ, the safety filter solves:

        u* = argmin_{u ∈ U}  ½ ‖u - u_AI‖²_Q          (QP-CBF)
             subject to:  Lf h(x) + Lg h(x) u ≥ -α(h(x))
                          u_min ≤ u ≤ u_max

THEOREM 2 (KKT Optimality of CBF-QP):
    The CBF-QP (QP-CBF) is a strictly convex QP. Its KKT conditions are:
        u* - u_AI + λ · Lg h(x)ᵀ + μ_upper - μ_lower = 0   [stationarity]
        λ · [Lf h(x) + Lg h(x) u* + α(h(x))] = 0            [complementarity]
        λ ≥ 0                                                  [dual feasibility]

    Therefore u* is the unique minimum-norm correction to u_AI satisfying safety.

COROLLARY (Minimal Perturbation):
    The CBF-QP produces the smallest possible change to u_AI:
        u* = argmin_{v ∈ Safe(x)} ‖v - u_AI‖₂

    This guarantees that robot performance is maximally preserved while
    enforcing safety — exactly the correct trade-off for AI-commanded robots.

─────────────────────────────────────────────────────────────────────────────
§3  CIRCULAR OBSTACLE CBF — EXPLICIT DERIVATION
─────────────────────────────────────────────────────────────────────────────

BARRIER FUNCTION for obstacle at (ox, oy) with radius r_obs, robot radius r_r:

    h(x) = (px - ox)² + (py - oy)² - (r_obs + r_r)²

Gradient:
    ∂h/∂x = [2(px - ox), 2(py - oy), 0]

Lie Derivatives for unicycle ẋ = [v cos θ, v sin θ, ω]:
    Lf h(x) = ∂h/∂x · f(x) = 0  (no drift in kinematic model)

    Lg h(x) = ∂h/∂x · g(x)
             = [2(px-ox), 2(py-oy), 0] · [[cosθ, 0], [sinθ, 0], [0, 1]]
             = [2(px-ox)cosθ + 2(py-oy)sinθ,  0]

CBF constraint becomes:
    [2(px-ox)cosθ + 2(py-oy)sinθ] · v ≥ -γ h(x)

This is a LINEAR constraint in v (can be solved analytically!).

─────────────────────────────────────────────────────────────────────────────
§4  LYAPUNOV STABILITY — DEFINITION AND VERIFICATION
─────────────────────────────────────────────────────────────────────────────

CANDIDATE LYAPUNOV FUNCTION:
    V(x) = ½ ‖x - x_goal‖²

STABILITY CONDITION (exponential convergence):
    V̇(x) ≤ -ε · ‖x - x_goal‖²  for some ε > 0

Equivalently:
    (x - x_goal)ᵀ · ẋ ≤ -2ε · V(x)

If this holds globally, the equilibrium x_goal is exponentially stable.

─────────────────────────────────────────────────────────────────────────────
§5  REFERENCES
─────────────────────────────────────────────────────────────────────────────

[1] Ames, A.D., Xu, X., Grizzle, J.W., Tabuada, P. (2017). Control barrier
    function based quadratic programs for safety critical systems. IEEE TAC.

[2] Ames, A.D., Coogan, S., Egerstedt, M., et al. (2019). Control barrier
    functions: Theory and applications. ECC 2019.

[3] Zafar, A.B. (2026). Principia Robotica: World-first unified MCP+CBF-QP
    agentic safety engine. GitHub.
    https://github.com/EngineerAbdullahBinZafar/principia-robotica

─────────────────────────────────────────────────────────────────────────────
END OF MATHEMATICAL FOUNDATIONS
─────────────────────────────────────────────────────────────────────────────
"""

if __name__ == "__main__":
    print(THEORY)
