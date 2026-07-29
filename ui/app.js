/* ==========================================================================
   Principia Robotica — Real-Time CBF-QP Web Simulator & Telemetry Engine
   Pure Vanilla JavaScript — 60 FPS HTML5 Canvas + Control Theory Math
   Author: Abdullah Bin Zafar
   ========================================================================== */

(function () {
    "use strict";

    // ── Global Canvas & State ─────────────────────────────────────────────
    const canvas = document.getElementById("sim-canvas");
    const ctx = canvas.getContext("2d");

    // Coordinate conversion (Canvas pixels <-> World meters)
    // World origin (0,0) is at Canvas center (400, 275)
    const SCALE = 60.0; // 60 pixels = 1.0 meter
    const CENTER_X = canvas.width / 2;
    const CENTER_Y = canvas.height / 2;

    function worldToCanvas(wx, wy) {
        return {
            x: CENTER_X + wx * SCALE,
            y: CENTER_Y - wy * SCALE, // Invert Y
        };
    }

    function canvasToWorld(cx, cy) {
        return {
            x: (cx - CENTER_X) / SCALE,
            y: (CENTER_Y - cy) / SCALE,
        };
    }

    // ── Simulation Engine State ───────────────────────────────────────────
    const state = {
        mode: "unicycle", // unicycle, quadrotor, swarm, dynamic
        robot: {
            x: -2.0,
            y: 0.0,
            theta: 0.0, // radians
            radius: 0.25, // meters
            v: 0.0,
            omega: 0.0,
        },
        quad: {
            z: 1.5,
            vz: 0.0,
            z_min: 0.4,
        },
        swarm: [
            { id: "r1", x: -2.0, y: 1.0, theta: 0.0, v: 0.5, omega: 0.1, color: "#38bdf8" },
            { id: "r2", x: -2.0, y: -1.0, theta: 0.0, v: 0.5, omega: -0.1, color: "#10b981" },
            { id: "r3", x: -3.0, y: 0.0, theta: 0.0, v: 0.6, omega: 0.0, color: "#a78bfa" },
        ],
        dynamicObs: {
            x: 1.5,
            y: 1.2,
            vx: -0.4,
            vy: -0.3,
            radius: 0.5,
        },
        obstacles: [
            { x: 0.0, y: 0.0, radius: 0.6, isDragging: false },
            { x: 1.8, y: -1.2, radius: 0.5, isDragging: false },
        ],
        goal: { x: 3.5, y: 0.0 },
        aiCommand: { v: 1.5, omega: 0.0 },
        cbfGamma: 1.5,
        draggedObs: null,
    };

    // ── Core CBF-QP Math Engine (Pure JS Implementation) ─────────────────

    /**
     * Solves CBF-QP for Circular Obstacle + Unicycle Kinematic Model:
     *   min ½ (v - v_AI)² + ½ (w - w_AI)²
     *   s.t. 2(px-ox)cosθ·v + 2(py-oy)sinθ·v ≥ -γ h(x)
     *        v_min ≤ v ≤ v_max, w_min ≤ w ≤ w_max
     */
    function solveCBFQP(robotState, aiCmd, obstacles, gamma) {
        const t0 = performance.now();
        const rx = robotState.x;
        const ry = robotState.y;
        const theta = robotState.theta;

        const v_AI = aiCmd.v;
        const w_AI = aiCmd.omega;

        const v_min = -0.5, v_max = 1.5;
        const w_min = -2.5, w_max = 2.5;

        let v_safe = Math.max(v_min, Math.min(v_max, v_AI));
        let w_safe = Math.max(w_min, Math.min(w_max, w_AI));

        let minH = Infinity;
        let cbfMargins = [];
        let modified = false;

        for (let obs of obstacles) {
            const dx = rx - obs.x;
            const dy = ry - obs.y;
            const r_safe = obs.radius + robotState.radius;
            const h = dx * dx + dy * dy - r_safe * r_safe;
            cbfMargins.push(h);

            if (h < minH) minH = h;

            // CBF Constraint Linear Coefficient for v:
            // Lg h · u = [2dx cosθ + 2dy sinθ] v
            const Lg_h_v = 2.0 * dx * Math.cos(theta) + 2.0 * dy * Math.sin(theta);
            const rhs = -gamma * h;

            // Check if constraint is violated by v_safe
            const currentConstraintVal = Lg_h_v * v_safe;
            if (currentConstraintVal < rhs - 1e-6) {
                modified = true;
                // If Lg_h_v > 0, moving forward increases h (safe).
                // If Lg_h_v < 0, moving forward decreases h (unsafe).
                if (Math.abs(Lg_h_v) > 1e-6) {
                    const v_bound = rhs / Lg_h_v;
                    if (Lg_h_v < 0) {
                        // Must restrict v ≤ v_bound (slow down or stop)
                        v_safe = Math.min(v_safe, Math.max(v_min, v_bound));
                    } else {
                        // Lg_h_v > 0: Must enforce v ≥ v_bound
                        v_safe = Math.max(v_safe, Math.min(v_max, v_bound));
                    }
                }
            }
        }

        // Clamp to limits
        v_safe = Math.max(v_min, Math.min(v_max, v_safe));
        w_safe = Math.max(w_min, Math.min(w_max, w_safe));

        const deltaV = v_safe - v_AI;
        const deltaW = w_safe - w_AI;
        const l2Norm = Math.sqrt(deltaV * deltaV + deltaW * deltaW);

        const t1 = performance.now();

        return {
            v_safe: v_safe,
            w_safe: w_safe,
            minH: minH === Infinity ? 1.0 : minH,
            cbfMargins: cbfMargins,
            modified: modified || l2Norm > 1e-4,
            l2Norm: l2Norm,
            solveTimeMs: (t1 - t0),
        };
    }

    // ── UI Elements ───────────────────────────────────────────────────────
    const sliderV = document.getElementById("slider-v");
    const sliderOmega = document.getElementById("slider-omega");
    const sliderGamma = document.getElementById("slider-gamma");

    const lblV = document.getElementById("lbl-v");
    const lblOmega = document.getElementById("lbl-omega");
    const lblGamma = document.getElementById("lbl-gamma");

    const hudPose = document.getElementById("hud-pose");
    const hudStatus = document.getElementById("hud-status");
    const hudSolveTime = document.getElementById("hud-solve-time");

    const cbfMarginVal = document.getElementById("cbf-margin-val");
    const gaugeBarInner = document.getElementById("gauge-bar-inner");

    const valVAi = document.getElementById("val-v-ai");
    const valWAi = document.getElementById("val-w-ai");
    const valVSafe = document.getElementById("val-v-safe");
    const valWSafe = document.getElementById("val-w-safe");
    const valL2Norm = document.getElementById("val-l2-norm");
    const valModifiedBadge = document.getElementById("val-modified-badge");

    const valLyapunovV = document.getElementById("val-lyapunov-v");
    const valLyapunovDv = document.getElementById("val-lyapunov-dv");

    // ── User Input Event Listeners ────────────────────────────────────────
    sliderV.addEventListener("input", (e) => {
        state.aiCommand.v = parseFloat(e.target.value);
        lblV.textContent = state.aiCommand.v.toFixed(2) + " m/s";
    });

    sliderOmega.addEventListener("input", (e) => {
        state.aiCommand.omega = parseFloat(e.target.value);
        lblOmega.textContent = state.aiCommand.omega.toFixed(2) + " rad/s";
    });

    sliderGamma.addEventListener("input", (e) => {
        state.cbfGamma = parseFloat(e.target.value);
        lblGamma.textContent = state.cbfGamma.toFixed(1);
    });

    document.getElementById("btn-reset-robot").addEventListener("click", () => {
        state.robot.x = -2.5;
        state.robot.y = 0.0;
        state.robot.theta = 0.0;
    });

    document.getElementById("btn-clear-obs").addEventListener("click", () => {
        state.obstacles = [];
    });

    document.getElementById("btn-add-obs").addEventListener("click", () => {
        state.obstacles.push({
            x: (Math.random() - 0.5) * 4.0,
            y: (Math.random() - 0.5) * 3.0,
            radius: 0.4 + Math.random() * 0.3,
        });
    });

    // Mode Selector Buttons
    document.querySelectorAll(".btn-mode").forEach((btn) => {
        btn.addEventListener("click", (e) => {
            document.querySelectorAll(".btn-mode").forEach((b) => b.classList.remove("active"));
            btn.classList.add("active");
            state.mode = btn.dataset.mode;
        });
    });

    // Mouse Interaction for Dragging Obstacles
    canvas.addEventListener("mousedown", (e) => {
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const wPos = canvasToWorld(mx, my);

        for (let obs of state.obstacles) {
            const dx = wPos.x - obs.x;
            const dy = wPos.y - obs.y;
            if (Math.sqrt(dx * dx + dy * dy) <= obs.radius + 0.2) {
                state.draggedObs = obs;
                break;
            }
        }
    });

    canvas.addEventListener("mousemove", (e) => {
        if (state.draggedObs) {
            const rect = canvas.getBoundingClientRect();
            const mx = e.clientX - rect.left;
            const my = e.clientY - rect.top;
            const wPos = canvasToWorld(mx, my);
            state.draggedObs.x = wPos.x;
            state.draggedObs.y = wPos.y;
        }
    });

    window.addEventListener("mouseup", () => {
        state.draggedObs = null;
    });

    // ── Main Render & Physics Simulation Loop ─────────────────────────────
    let lastTime = performance.now();

    function renderLoop(now) {
        const dt = Math.min(0.05, (now - lastTime) / 1000.0);
        lastTime = now;

        // 1. Solve CBF-QP
        const qpResult = solveCBFQP(state.robot, state.aiCommand, state.obstacles, state.cbfGamma);

        // 2. Physics Step Integration (Unicycle model)
        if (state.mode === "unicycle") {
            const v = qpResult.v_safe;
            const w = qpResult.w_safe;

            state.robot.x += v * Math.cos(state.robot.theta) * dt;
            state.robot.y += v * Math.sin(state.robot.theta) * dt;
            state.robot.theta += w * dt;
        } else if (state.mode === "dynamic") {
            // Move dynamic obstacle
            state.dynamicObs.x += state.dynamicObs.vx * dt;
            state.dynamicObs.y += state.dynamicObs.vy * dt;
            if (Math.abs(state.dynamicObs.x) > 3.5) state.dynamicObs.vx *= -1;
            if (Math.abs(state.dynamicObs.y) > 2.5) state.dynamicObs.vy *= -1;

            const v = qpResult.v_safe;
            const w = qpResult.w_safe;
            state.robot.x += v * Math.cos(state.robot.theta) * dt;
            state.robot.y += v * Math.sin(state.robot.theta) * dt;
            state.robot.theta += w * dt;
        } else if (state.mode === "swarm") {
            // Move swarm robots
            for (let r of state.swarm) {
                r.x += r.v * Math.cos(r.theta) * dt;
                r.y += r.v * Math.sin(r.theta) * dt;
                r.theta += r.omega * dt;

                // Simple wall bounce
                if (Math.abs(r.x) > 3.8) r.theta = Math.PI - r.theta;
                if (Math.abs(r.y) > 2.8) r.theta = -r.theta;
            }
        }

        // Keep robot in bounds
        if (Math.abs(state.robot.x) > 4.5) state.robot.x = Math.sign(state.robot.x) * 4.5;
        if (Math.abs(state.robot.y) > 3.0) state.robot.y = Math.sign(state.robot.y) * 3.0;

        // 3. Draw Canvas Graphics
        drawScene(qpResult);

        // 4. Update Telemetry Panels
        updateTelemetry(qpResult);

        requestAnimationFrame(renderLoop);
    }

    // ── Canvas Rendering Engine ───────────────────────────────────────────
    function drawScene(qpResult) {
        // Clear
        ctx.fillStyle = "#040711";
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Draw Grid Lines (0.5m spacing)
        ctx.strokeStyle = "rgba(56, 189, 248, 0.08)";
        ctx.lineWidth = 1;

        for (let x = -5; x <= 5; x += 0.5) {
            const p1 = worldToCanvas(x, -3.5);
            const p2 = worldToCanvas(x, 3.5);
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.stroke();
        }

        for (let y = -3.5; y <= 3.5; y += 0.5) {
            const p1 = worldToCanvas(-6, y);
            const p2 = worldToCanvas(6, y);
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.stroke();
        }

        // Draw Goal Post
        const gPos = worldToCanvas(state.goal.x, state.goal.y);
        ctx.fillStyle = "#10b981";
        ctx.beginPath();
        ctx.arc(gPos.x, gPos.y, 8, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "rgba(16, 185, 129, 0.4)";
        ctx.lineWidth = 12;
        ctx.beginPath();
        ctx.arc(gPos.x, gPos.y, 16, 0, Math.PI * 2);
        ctx.stroke();

        // Draw Obstacles (with CBF Safety Boundary Rings)
        const obsList = state.mode === "dynamic" ? [...state.obstacles, state.dynamicObs] : state.obstacles;

        for (let obs of obsList) {
            const oPos = worldToCanvas(obs.x, obs.y);
            const rPx = obs.radius * SCALE;
            const rSafePx = (obs.radius + state.robot.radius) * SCALE;

            // Physical Obstacle
            ctx.fillStyle = "rgba(244, 63, 94, 0.35)";
            ctx.strokeStyle = "#f43f5e";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(oPos.x, oPos.y, rPx, 0, Math.PI * 2);
            ctx.fill();
            ctx.stroke();

            // CBF Safety Invariance Boundary h(x)=0 Ring
            ctx.strokeStyle = "rgba(245, 158, 11, 0.6)";
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.arc(oPos.x, oPos.y, rSafePx, 0, Math.PI * 2);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // Draw Dynamic Obstacle Velocity Vector
        if (state.mode === "dynamic") {
            const dPos = worldToCanvas(state.dynamicObs.x, state.dynamicObs.y);
            ctx.strokeStyle = "#f43f5e";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(dPos.x, dPos.y);
            ctx.lineTo(dPos.x + state.dynamicObs.vx * SCALE * 0.8, dPos.y - state.dynamicObs.vy * SCALE * 0.8);
            ctx.stroke();
        }

        // Draw Trajectory Prediction Line (Forward Kinematic Simulator)
        drawPredictedTrajectory(qpResult);

        // Draw Main Robot
        const rPos = worldToCanvas(state.robot.x, state.robot.y);
        const robotR = state.robot.radius * SCALE;

        // Robot Safety Glow
        ctx.fillStyle = qpResult.modified ? "rgba(245, 158, 11, 0.25)" : "rgba(56, 189, 248, 0.25)";
        ctx.beginPath();
        ctx.arc(rPos.x, rPos.y, robotR + 6, 0, Math.PI * 2);
        ctx.fill();

        // Robot Body
        ctx.fillStyle = qpResult.modified ? "#f59e0b" : "#38bdf8";
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(rPos.x, rPos.y, robotR, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Robot Heading Indicator
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(rPos.x, rPos.y);
        ctx.lineTo(
            rPos.x + Math.cos(state.robot.theta) * (robotR + 10),
            rPos.y - Math.sin(state.robot.theta) * (robotR + 10)
        );
        ctx.stroke();

        // Draw AI Velocity Vector (Violet Arrow)
        ctx.strokeStyle = "rgba(139, 92, 246, 0.8)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(rPos.x, rPos.y);
        ctx.lineTo(
            rPos.x + Math.cos(state.robot.theta) * state.aiCommand.v * SCALE * 0.6,
            rPos.y - Math.sin(state.robot.theta) * state.aiCommand.v * SCALE * 0.6
        );
        ctx.stroke();

        // Draw CBF-QP Safe Velocity Vector (Cyan/Amber Arrow)
        ctx.strokeStyle = qpResult.modified ? "#f59e0b" : "#10b981";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(rPos.x, rPos.y);
        ctx.lineTo(
            rPos.x + Math.cos(state.robot.theta + qpResult.w_safe * 0.1) * qpResult.v_safe * SCALE * 0.6,
            rPos.y - Math.sin(state.robot.theta + qpResult.w_safe * 0.1) * qpResult.v_safe * SCALE * 0.6
        );
        ctx.stroke();

        // Draw Swarm Fleet Robots if in Swarm Mode
        if (state.mode === "swarm") {
            for (let bot of state.swarm) {
                const bPos = worldToCanvas(bot.x, bot.y);
                ctx.fillStyle = bot.color;
                ctx.beginPath();
                ctx.arc(bPos.x, bPos.y, state.robot.radius * SCALE, 0, Math.PI * 2);
                ctx.fill();
            }
        }
    }

    function drawPredictedTrajectory(qpResult) {
        let simX = state.robot.x;
        let simY = state.robot.y;
        let simTheta = state.robot.theta;

        ctx.strokeStyle = qpResult.modified ? "rgba(245, 158, 11, 0.7)" : "rgba(16, 185, 129, 0.7)";
        ctx.lineWidth = 2;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();

        const startPos = worldToCanvas(simX, simY);
        ctx.moveTo(startPos.x, startPos.y);

        for (let i = 0; i < 40; i++) {
            simX += qpResult.v_safe * Math.cos(simTheta) * 0.05;
            simY += qpResult.v_safe * Math.sin(simTheta) * 0.05;
            simTheta += qpResult.w_safe * 0.05;

            const p = worldToCanvas(simX, simY);
            ctx.lineTo(p.x, p.y);
        }

        ctx.stroke();
        ctx.setLineDash([]);
    }

    // ── Telemetry Update Function ─────────────────────────────────────────
    function updateTelemetry(qpResult) {
        const deg = ((state.robot.theta * 180 / Math.PI) % 360).toFixed(1);
        hudPose.textContent = `(${state.robot.x.toFixed(2)}m, ${state.robot.y.toFixed(2)}m, ${deg}°)`;
        hudSolveTime.textContent = qpResult.solveTimeMs.toFixed(3) + " ms";

        if (qpResult.modified) {
            hudStatus.textContent = "INTERCEPTED (MINIMAL PERTURBATION APPLIED)";
            hudStatus.className = "hud-value status-modified";
        } else {
            hudStatus.textContent = "POSITIVE INVARIANT (SAFE)";
            hudStatus.className = "hud-value status-safe";
        }

        // CBF Gauge
        const margin = qpResult.minH;
        cbfMarginVal.textContent = (margin >= 0 ? "+" : "") + margin.toFixed(4);

        if (margin < 0) {
            cbfMarginVal.style.color = "#f43f5e";
            gaugeBarInner.style.background = "#f43f5e";
        } else if (margin < 0.2) {
            cbfMarginVal.style.color = "#f59e0b";
            gaugeBarInner.style.background = "#f59e0b";
        } else {
            cbfMarginVal.style.color = "#10b981";
            gaugeBarInner.style.background = "linear-gradient(90deg, #10b981, #38bdf8)";
        }

        const pct = Math.max(0, Math.min(100, (margin + 0.5) * 50));
        gaugeBarInner.style.width = pct + "%";

        // Commands
        valVAi.textContent = state.aiCommand.v.toFixed(3) + " m/s";
        valWAi.textContent = state.aiCommand.omega.toFixed(3) + " rad/s";

        valVSafe.textContent = qpResult.v_safe.toFixed(3) + " m/s";
        valWSafe.textContent = qpResult.w_safe.toFixed(3) + " rad/s";

        valL2Norm.textContent = qpResult.l2Norm.toFixed(4);

        if (qpResult.modified) {
            valModifiedBadge.textContent = "INTERCEPTED";
            valModifiedBadge.className = "badge badge-warning";
        } else {
            valModifiedBadge.textContent = "UNALTERED";
            valModifiedBadge.className = "badge badge-success";
        }

        // Lyapunov
        const dx = state.robot.x - state.goal.x;
        const dy = state.robot.y - state.goal.y;
        const V = 0.5 * (dx * dx + dy * dy);
        valLyapunovV.textContent = V.toFixed(4);

        const dV = dx * (qpResult.v_safe * Math.cos(state.robot.theta)) + dy * (qpResult.v_safe * Math.sin(state.robot.theta));
        valLyapunovDv.textContent = dV.toFixed(4) + (dV < 0 ? " (CONVERGING)" : " (PAUSED)");
    }

    // Start loop
    requestAnimationFrame(renderLoop);

})();
