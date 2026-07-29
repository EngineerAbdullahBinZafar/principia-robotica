# Principia Robotica — Mathematical Theory and Formal Proofs

**Author:** Abdullah Bin Zafar  
**Institution:** UET Lahore, Pakistan  
**Date:** July 29, 2026  
**License:** MIT

---

## §1 Control Barrier Functions — Formal Definition

### Dynamical System

Consider a control-affine nonlinear dynamical system:

$$\dot{x} = f(x) + g(x)u, \quad x \in \mathbb{R}^n, \quad u \in \mathcal{U} \subseteq \mathbb{R}^m$$

where $f: \mathbb{R}^n \to \mathbb{R}^n$ (drift) and $g: \mathbb{R}^n \to \mathbb{R}^{n \times m}$ (control gain) are locally Lipschitz continuous.

### Safe Set

Define the safe set via a continuously differentiable function $h: \mathbb{R}^n \to \mathbb{R}$:

$$\mathcal{C} = \{x \in \mathbb{R}^n : h(x) \geq 0\}$$
$$\partial\mathcal{C} = \{x \in \mathbb{R}^n : h(x) = 0\}$$

### Definition (Control Barrier Function)

$h$ is a **Control Barrier Function (CBF)** for system $(\dot{x} = f(x) + g(x)u)$ on $\mathcal{C}$ if there exists a class-$\mathcal{K}$ function $\alpha$ such that for all $x \in \mathcal{C}$:

$$\sup_{u \in \mathcal{U}} \left[L_f h(x) + L_g h(x) u\right] \geq -\alpha(h(x))$$

where the **Lie derivatives** are:

$$L_f h(x) = \frac{\partial h}{\partial x} \cdot f(x), \quad L_g h(x) = \frac{\partial h}{\partial x} \cdot g(x)$$

---

## §2 Forward Invariance Theorem

### Theorem 1 (Ames et al., 2017)

If $h$ is a valid CBF for system $(\dot{x} = f(x) + g(x)u)$ and a locally Lipschitz control law $u(x)$ satisfies:

$$L_f h(x) + L_g h(x) u(x) \geq -\alpha(h(x)) \quad \forall x \in \mathcal{C}$$

Then $\mathcal{C}$ is **forward invariant**:

$$x(0) \in \mathcal{C} \implies x(t) \in \mathcal{C} \quad \forall t \geq 0$$

### Proof

Define $V(t) = h(x(t))$ along closed-loop trajectories. Then:

$$\dot{V}(t) = L_f h(x) + L_g h(x) u(x) \geq -\alpha(h(x)) = -\alpha(V(t))$$

By the **Comparison Lemma** (Khalil, 2002, Lemma 3.4), there exists a class-$\mathcal{KL}$ function $\beta$ such that:

$$V(t) \geq \beta(V(0), t) > 0 \quad \text{whenever } V(0) > 0$$

Therefore $h(x(t)) \geq 0$ for all $t \geq 0$, establishing forward invariance. $\square$

---

## §3 CBF-QP Safety Filter — Optimality Proof

### Definition (CBF-QP Safety Filter)

Given desired control $u_{\text{AI}} \in \mathbb{R}^m$ from an AI agent, the safety filter solves:

$$u^* = \underset{u \in \mathcal{U}}{\arg\min} \quad \frac{1}{2}\|u - u_{\text{AI}}\|^2_Q$$

$$\text{subject to:} \quad L_f h(x) + L_g h(x) u \geq -\alpha(h(x))$$
$$u_{\min} \leq u \leq u_{\max}$$

### Theorem 2 (KKT Optimality)

The CBF-QP is **strictly convex**. Its KKT optimality conditions are:

1. **Stationarity:** $u^* - u_{\text{AI}} + \lambda \cdot L_g h(x)^T + \mu_u - \mu_l = 0$
2. **Primal Feasibility:** $L_f h(x) + L_g h(x) u^* \geq -\alpha(h(x))$
3. **Dual Feasibility:** $\lambda \geq 0$, $\mu_u \geq 0$, $\mu_l \geq 0$
4. **Complementarity:** $\lambda \cdot [L_f h + L_g h \cdot u^* + \alpha(h)] = 0$

These conditions are necessary and sufficient, and the strictly convex objective guarantees **uniqueness** of $u^*$.

### Corollary (Minimal Perturbation)

The CBF-QP produces the **minimum-norm** deviation from $u_{\text{AI}}$:

$$u^* = \underset{v \in \mathcal{S}(x)}{\arg\min} \|v - u_{\text{AI}}\|_2$$

where $\mathcal{S}(x) = \{u : L_f h + L_g h \cdot u \geq -\alpha(h), u_{\min} \leq u \leq u_{\max}\}$ is the safe control set.

This guarantees **maximum performance preservation** while enforcing safety — the correct engineering trade-off for AI-commanded robots.

---

## §4 Circular Obstacle CBF — Explicit Derivation

### Setup

Obstacle at $(o_x, o_y)$ with radius $r_{\text{obs}}$; robot with safety radius $r_r$.

**Barrier function:**

$$h(x) = (p_x - o_x)^2 + (p_y - o_y)^2 - (r_{\text{obs}} + r_r)^2$$

**Gradient:**

$$\frac{\partial h}{\partial x} = \begin{bmatrix} 2(p_x - o_x) \\ 2(p_y - o_y) \\ 0 \end{bmatrix}^T$$

### Lie Derivatives for Unicycle Model

The unicycle dynamics $\dot{x} = [v\cos\theta, v\sin\theta, \omega]^T$ decompose as:

$$f(x) = \mathbf{0}, \quad g(x) = \begin{bmatrix} \cos\theta & 0 \\ \sin\theta & 0 \\ 0 & 1 \end{bmatrix}$$

Therefore:

$$L_f h(x) = 0$$

$$L_g h(x) = \begin{bmatrix} 2(p_x-o_x)\cos\theta + 2(p_y-o_y)\sin\theta & 0 \end{bmatrix}$$

The CBF constraint reduces to a **single linear inequality in $v$**:

$$\underbrace{\left[2(p_x-o_x)\cos\theta + 2(p_y-o_y)\sin\theta\right]}_{\text{signed projection}} \cdot v \geq -\gamma h(x)$$

This has a **closed-form solution**, meaning the QP reduces to a simple scalar problem for the single-obstacle case — enabling sub-microsecond solutions.

---

## §5 Lyapunov Stability

### Candidate Function

$$V(x) = \frac{1}{2}\|x - x_{\text{goal}}\|^2 \geq 0$$

### Stability Condition

For exponential convergence with rate $\epsilon > 0$:

$$\dot{V}(x) = (x - x_{\text{goal}})^T \dot{x} \leq -\epsilon \|x - x_{\text{goal}}\|^2 = -2\epsilon V(x)$$

If this holds globally, then by Gronwall's inequality:

$$V(x(t)) \leq V(x(0)) e^{-2\epsilon t}$$

So $\|x(t) - x_{\text{goal}}\| \leq \|x(0) - x_{\text{goal}}\| e^{-\epsilon t}$ — **exponential convergence**.

---

## §6 References

1. **Ames, A.D., Xu, X., Grizzle, J.W., Tabuada, P.** (2017). Control barrier function based quadratic programs for safety critical systems. *IEEE Trans. Autom. Control*, 62(8), 3861–3876.

2. **Ames, A.D., Coogan, S., Egerstedt, M., et al.** (2019). Control barrier functions: Theory and applications. *ECC 2019*.

3. **Khalil, H.K.** (2002). *Nonlinear Systems* (3rd ed.). Prentice Hall.

4. **Zafar, A.B.** (2026). Principia Robotica: World-first unified MCP+CBF-QP agentic safety engine. *GitHub*. https://github.com/EngineerAbdullahBinZafar/principia-robotica
