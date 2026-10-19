# Custom Kinematics Solvers

Pinocchio is **optional**. The FIC nodes (`fic_control`, `fic_control`) talk to
an abstract solver interface — `fic::KinematicsSolver` — and the concrete backend is
selected **per robot** via the `kinematics_solver:` key in
`fic_library/config/robots_config.yaml`. This document explains the architecture, how
to write and register your own solver, and what happens when no solver is available.

```
 fic_control
        │  (per-robot key: kinematics_solver)
        ▼
 fic::createKinematicsSolver(name)          fic_library/include/fic/solvers/solver_factory.hpp
        │
        ├── "pinocchio"  → fic::PinocchioSolver   (built-in, requires pinocchio at build time)
        ├── "pinocchio_closed_loop" → fic::ClosedLoopPinocchioSolver  (built-in, parallel robots)
        ├── "none" / ""  → nullptr                (degraded mode: joint-space only)
        └── "<yours>"    → your fic::KinematicsSolver implementation
```

## Selecting a solver

Per robot in `fic_library/config/robots_config.yaml`:

```yaml
robots:
  Franka_FR3:
    urdf_path: menagerie/model/Franka_FR3/URDF/Franka_FR3.urdf
    ee_frame_name: ee_link
    kinematics_solver: pinocchio   # "pinocchio" (default) | "pinocchio_closed_loop" | "none" | your registered name
```

The key is optional — omitting it means `pinocchio`.

> **GUI note:** Robot Configuration → Save writes `robots_config.yaml` from the
> Studio's own list (`fic_studio/fic_studio_robots.yaml`). `kinematics_solver`
> survives Save; if you hand-edit `robots_config.yaml`, set the key in
> `fic_studio_robots.yaml` too.

## The interface

Header: `fic_library/include/fic/kinematics_solver.hpp` (Eigen-only — no ROS, no
pinocchio — so your solver is unit-testable outside a ROS workspace).

| Method | Purpose | May return "not available"? |
|---|---|---|
| `name()` | identifier used in logs | — |
| `loadModel(urdf_path)` | parse robot model; `false` on failure (never throw) | yes — `false` + `lastError()` |
| `isLoaded()` / `nq()` / `nv()` | model state and dimensions | — |
| `lowerPositionLimit(i)` / `upperPositionLimit(i)` | joint position limits, `i` in `[0, nv)` | — |
| `jointConfigIndex(name)` | q-vector index of a named joint | yes — `-1` |
| `hasFrame(name)` | does the named frame exist | — |
| `updateKinematics(q)` | FK + frame placements on persistent state | — |
| `cachedFramePose(name, out)` | frame pose after last `updateKinematics` | yes — `false` |
| `frameJacobian(q, name, ref, out)` | 6×nv Jacobian (`Local` or `LocalWorldAligned`) | yes — `false` |
| `gravity(q)` | generalized gravity torques (size nv) | zero vector is acceptable |
| `framePose(q, name, out)` | one-off FK with local scratch state | yes — `false` |
| `solveIK(q_start, target, frame, opts)` | iterative IK; returns best iterate even on failure | `success=false` |

### Threading contract (important)

- **Cached path** — `updateKinematics`, `cachedFramePose`, `frameJacobian`,
  `gravity`: called only from the 1 kHz control-loop thread. May share persistent
  scratch state. Not required to be thread-safe.
- **Scratch path** — `framePose`, `solveIK`: `const` and **must be safe to call
  concurrently** with the cached path — both nodes run `solveIK` off the control
  thread while the control loop spins: the simulation node in a detached thread per
  Cartesian target, the hardware node on an IK worker thread that solves the newest
  pending target (followed by `framePose` for the elbow). Use function-local state only.

### IKOptions semantics

| Field | Meaning |
|---|---|
| `tolerance` | convergence threshold on the 6D pose-error norm |
| `max_iterations`, `dt`, `damping` | CLIK loop parameters |
| `joint_based` | `true` = error anchored at the frame's **parent joint** (hardware node uses this); `false` = at the **frame** itself in the Local convention (simulation node) |
| `max_joint_velocity` | `> 0` clamps the per-iteration joint-velocity norm (sim uses `0.5`) |

Your solver may ignore `joint_based` if it has a single IK formulation — it exists so
the built-in solver reproduces both nodes' historical behavior exactly.

## Writing your own solver

A minimal pure-Eigen example — a 2-link planar arm (illustrative; real solvers
would parse the URDF in `loadModel`):

```cpp
// fic_library/include/fic/solvers/two_link_solver.hpp
#pragma once
#include <fic/kinematics_solver.hpp>

namespace fic {

class TwoLinkSolver final : public KinematicsSolver {
public:
    std::string name() const override { return "two_link"; }

    bool loadModel(const std::string& /*urdf_path*/) override {
        loaded_ = true;          // fixed geometry: L1 = L2 = 0.5 m
        return true;
    }
    bool isLoaded() const override { return loaded_; }
    int nq() const override { return 2; }
    int nv() const override { return 2; }

    double lowerPositionLimit(int) const override { return -M_PI; }
    double upperPositionLimit(int) const override { return  M_PI; }
    int jointConfigIndex(const std::string& j) const override {
        return j == "joint1" ? 0 : j == "joint2" ? 1 : -1;
    }
    bool hasFrame(const std::string& f) const override { return f == "ee_link"; }

    void updateKinematics(const Eigen::VectorXd& q) override { q_cached_ = q; }

    bool cachedFramePose(const std::string& f, Eigen::Isometry3d& out) const override {
        return framePose(q_cached_, f, out);
    }

    bool framePose(const Eigen::VectorXd& q, const std::string& f,
                   Eigen::Isometry3d& out) const override {
        if (f != "ee_link" || q.size() < 2) return false;
        out = Eigen::Isometry3d::Identity();
        out.translation() << L1*std::cos(q[0]) + L2*std::cos(q[0]+q[1]),
                             L1*std::sin(q[0]) + L2*std::sin(q[0]+q[1]), 0.0;
        out.linear() = Eigen::AngleAxisd(q[0]+q[1], Eigen::Vector3d::UnitZ()).matrix();
        return true;
    }

    bool frameJacobian(const Eigen::VectorXd& q, const std::string& f,
                       JacobianFrame, Matrix6x& out) override {
        if (f != "ee_link" || q.size() < 2) return false;
        out.setZero(6, 2);
        const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
        const double s12 = std::sin(q[0]+q[1]), c12 = std::cos(q[0]+q[1]);
        out(0,0) = -L1*s1 - L2*s12;  out(0,1) = -L2*s12;
        out(1,0) =  L1*c1 + L2*c12;  out(1,1) =  L2*c12;
        out(5,0) = 1.0;              out(5,1) = 1.0;
        return true;
    }

    Eigen::VectorXd gravity(const Eigen::VectorXd& q) override {
        return Eigen::VectorXd::Zero(q.size());   // zero feedforward is legal
    }

    IKResult solveIK(const Eigen::VectorXd& q_start, const Eigen::Isometry3d& target,
                     const std::string& frame, const IKOptions& opts) const override {
        IKResult r; r.q = q_start;
        if (!hasFrame(frame)) return r;
        Eigen::VectorXd q = q_start;
        for (int i = 0; i < opts.max_iterations; ++i) {
            Eigen::Isometry3d cur; framePose(q, frame, cur);
            Eigen::Vector2d err = (target.translation() - cur.translation()).head<2>();
            if (err.norm() < opts.tolerance) { r.success = true; break; }
            Matrix6x J; const_cast<TwoLinkSolver*>(this)->frameJacobian(q, frame, JacobianFrame::Local, J);
            Eigen::Matrix2d Jp = J.topLeftCorner<2,2>();
            q += opts.dt * Jp.transpose() *
                 (Jp*Jp.transpose() + opts.damping*Eigen::Matrix2d::Identity()).ldlt().solve(err);
            r.iterations = i + 1;
            r.error_norm = err.norm();
        }
        r.q = q;
        return r;
    }

private:
    static constexpr double L1 = 0.5, L2 = 0.5;
    bool loaded_ = false;
    Eigen::VectorXd q_cached_;
};

}  // namespace fic
```

### Registering it

Open `fic_library/include/fic/solvers/solver_factory.hpp` and fill in the two
`USER SOLVERS` markers:

```cpp
// ── USER SOLVERS: include your solver header here ───────────────────────────
#include <fic/solvers/two_link_solver.hpp>
...
// ── USER SOLVERS: register your solver name here, then rebuild ──────────────
if (name == "two_link") return std::make_unique<TwoLinkSolver>();
```

Rebuild the two consumers:

```bash
cd fic_ws
colcon build --packages-select fic_control
```

Then point a robot at it:

```yaml
    kinematics_solver: two_link
```

## Building without pinocchio

Both packages auto-detect pinocchio (`find_package(pinocchio QUIET)` behind
`option(FIC_WITH_PINOCCHIO ON)`):

- pinocchio **found** → the built-in `"pinocchio"` solver is compiled in (default).
- pinocchio **absent** (or `--cmake-args -DFIC_WITH_PINOCCHIO=OFF`) → the build
  still succeeds; requesting `kinematics_solver: pinocchio` at runtime logs an
  error and enters degraded mode.

`package.xml` still declares `<depend>pinocchio</depend>` so rosdep installs it by
default; colcon itself does not require the key to build. Use rosdep's
`skip-keys` option (`rosdep install ... --skip-keys pinocchio`) on machines that
intentionally go without it.

## Degraded mode (no solver)

When `kinematics_solver: none`, the name is unknown, or the pinocchio build is off:

| Feature | Behavior |
|---|---|
| Joint-space modes 1–4, driver bridge, filters, ramp-down, torque guards, hard-fault | fully functional |
| Gravity compensation | **sim:** still works (MuJoCo publishes gravity); **hardware:** zero feedforward |
| Cartesian modes 5–8 | mode switch refused with an ERROR |
| Cartesian pose targets | ignored, throttled warning |
| EE pose / Jacobian publishers | silent (nothing published) |
| Jacobian-based wrench estimate (hardware) | skipped; the `wrench_from_api` path still publishes |
| Joint limits for the controller | read from the URDF XML directly, falling back to `joint_limits_deg` in the YAML, then ±2π |

### Safety: the Cartesian workspace guard fails open

Without a solver there is no forward kinematics, so the Cartesian workspace guard
(`eeTargetInsideZone`) and the J^T virtual-wall force **cannot run. They fail open**:
joint targets are accepted without the EE-position check. This matches the
pre-existing behavior when no model was loaded, and the alternative (fail closed)
would reject every joint target — bricking the only control modes degraded mode
supports. Joint-space safety limits and the per-joint torque hard-fault remain fully
enforced. A throttled warning is logged whenever a Cartesian zone is enabled without
a solver. **Do not rely on Cartesian safety zones when running degraded.**

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `unknown kinematics_solver 'x'` at startup | name not registered in `solver_factory.hpp` — check spelling, add the branch, rebuild |
| `solver 'pinocchio' requested but this build has FIC_WITH_PINOCCHIO=OFF` | pinocchio was missing at build time — install `ros-$ROS_DISTRO-pinocchio` and rebuild |
| `Failed to load robot model: ...` | `loadModel` failed — the message carries `lastError()`; check the `urdf_path` in the YAML (relative to `fic_ws/src/`) |
| `IK: Frame xyz not found.` / `EE frame not found!` | `ee_frame_name` / `elbow_frame_name` in the YAML doesn't exist in the URDF (or your solver's `hasFrame`) |
| Cartesian mode refused | you are in degraded mode — see the startup banner |
