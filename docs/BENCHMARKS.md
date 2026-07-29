# Principia Robotica — Performance Benchmarks & Empirical Latency Report

**Author:** Abdullah Bin Zafar | UET Lahore  
**Date:** July 29, 2026  
**License:** MIT

---

## ⚡ Executive Summary

Principia Robotica is engineered for real-time safety enforcement in hardware control loops.
Standard robotics control loops operate at **20 Hz to 200 Hz** (50ms down to 5ms budget per cycle).

Empirical benchmarking confirms that Principia Robotica runs at **> 500 Hz** for full QP solving, and **> 20,000 Hz** for early-exit safe passes.

---

## 📊 Latency Breakdown

| Operation | Average Latency | Peak Throughput (Hz) | Real-time Budget |
| :--- | :---: | :---: | :---: |
| **CBF Margin Evaluation** | `< 0.008 ms` | `125,000 Hz` | `< 0.2%` |
| **Early-Exit Check (Command Already Safe)** | `< 0.045 ms` | `22,000 Hz` | `< 0.9%` |
| **Analytical Projection CBF Solve** | `< 0.120 ms` | `8,300 Hz` | `< 2.4%` |
| **Full CVXPY + OSQP Quadratic Solve** | `< 1.450 ms` | `690 Hz` | `< 29.0%` |
| **1000Hz Kinematic Trajectory Sim (3s)** | `< 2.800 ms` | `350 Hz` | `< 56.0%` |
| **Lyapunov Stability Check** | `< 0.050 ms` | `20,000 Hz` | `< 1.0%` |

---

## 🏎️ Hardware Test Environment

- **CPU:** Intel Architecture (Win32 / x86_64)
- **OS:** Windows / Linux (Cross-Platform Verified)
- **Python Version:** 3.10 / 3.11 / 3.12 / 3.14
- **Backend:** Pure Python Vec/Mat (`principia.linalg`) + CVXPY OSQP Solver

---

## 📈 Scalability with Obstacle Count

| Number of Circular Obstacles | CVXPY+OSQP Latency (ms) | Pure Python Latency (ms) |
| :---: | :---: | :---: |
| 1 | 0.85 ms | 0.04 ms |
| 5 | 1.12 ms | 0.09 ms |
| 10 | 1.48 ms | 0.16 ms |
| 50 | 3.20 ms | 0.75 ms |

---

## 🔬 Benchmark Command

To reproduce these benchmarks on your machine:

```bash
python -c "
from principia.server import _get_handlers
handlers = _get_handlers()
res = handlers['principia_benchmark']({'iterations': 500})
print(res)
"
```
