# Safety Net Hardware Verification Protocol

**Project:** FIC Control Studio

This protocol checks, on **real hardware**, the safety nets of FIC Control
Studio. Each net is covered in simulation or by unit tests (the *Sim/unit
status* lines); this document is the step-by-step **expected-vs-observed**
procedure that confirms the same behaviour on a physical robot.

> The robot-side behaviour of these nets cannot be exercised in simulation: the
> simulation controller talks to MuJoCo over ROS, while the hardware controller
> talks to the vendor driver over shared memory.

---

## 0. Pre-flight — DO NOT SKIP

Before running ANY test below:

- [ ] A **physical emergency stop** is within reach and tested.
- [ ] Workspace is clear of people and obstacles; robot is bolted/secured.
- [ ] Start with **low stiffness** on the active control mode.
- [ ] One operator on the software E-STOP (Space / toolbar), one on the physical E-STOP.
- [ ] The correct robot model + `num_joints` are selected; `config.xml` matches the arm.
- [ ] First run each test at **reduced speed / small motions**, then at normal scale.
- [ ] Record firmware/driver versions and the robot serial in the sign-off table.

**Abort criteria (any test):** unexpected motion toward a person, torque spike you
can hear/feel, the arm leaving the intended zone, or any net failing to engage →
hit the physical E-STOP and stop the session.

---

## 1. Test environment

| Item | Value |
|---|---|
| Robot | __________ (model, serial) |
| Driver / firmware | __________ |
| Controller | `fic_control` (version: __________) |
| Station / robot id | `robot{N}` = __________ |
| Operators | __________ (software) / __________ (physical E-STOP) |
| Date / time | __________ |

Build the hardware controller from this branch before testing:

```bash
cd fic_ws
colcon build --packages-select fic_control
source install/setup.bash
```

---

## 2. Net-by-net procedures

Fill the **Observed** and **Pass?** cells during the run. A test passes only if
Observed matches Expected AND no abort criterion triggered.

### 2.1 Cartesian workspace zone — target clamp (Cartesian modes 5–8)

**What it does:** the controller clamps a commanded EE target to the enabled
Cartesian box before acting (`applyCartesianSafetyLimits`).
**Sim/unit status:** checked in simulation.

| # | Step | Expected | Observed | Pass? |
|---|---|---|---|---|
| a | Open **⊘ Safety**, set Z max = (a value ~0.10 m inside reach), enable Z, enable master. Banner reads green "SAFETY ZONES ACTIVE". | Limits broadcast; controller logs "Updated Cartesian safety limits". | | |
| b | In a Cartesian mode, command an EE target **above** Z max (e.g. +0.15 m past it). | The EE stops at Z max; it does **not** cross the boundary. | | |
| c | Repeat for X and Y min/max. | EE held inside the box on every enabled axis. | | |
| d | Disable the Z axis only; re-command past old Z max. | EE now passes the old Z limit (axis disabled), X/Y still clamped. | | |

### 2.2 Joint-mode EE-workspace guard (FK rejection)

**What it does:** in joint modes (1–4), each candidate joint target is run
through forward kinematics; if the resulting EE would leave the enabled Cartesian
box the joint target is **rejected** (last in-zone target held). Wires
`eeTargetInsideZone` → `isPositionInSafetyZone`.
**Sim/unit status:** checked in simulation.

| # | Step | Expected | Observed | Pass? |
|---|---|---|---|---|
| a | Enable a Z zone with the **current** EE just inside it. Switch to a **joint** control mode. | Banner green; ▸ marks "Joint angle bounds". | | |
| b | Jog / command joints so the EE *would* rise above Z max. | The arm **stops at the boundary** — the joint command that would exit is not executed; the EE stays inside the box. | | |
| c | Command joints that keep the EE inside the box. | Motion proceeds normally (guard only blocks exits). | | |
| d | Disable the Cartesian zone; repeat (b). | Joint motion now proceeds past the old boundary (guard inactive). | | |

> Note: this is *preventive* (blocks the command). It is independent of the
> reactive monitor in §2.5, which is the backstop if a zone is changed mid-motion.

### 2.3 Output-torque guard (finite + saturation)

**What it does:** before the torque reaches the robot it is forced finite
(`NaN/Inf → 0`) and clamped to the per-joint torque limit (fallback
`output_torque_limit_`, 150 N·m). In
hardware this guards the single `commanded_torque` shared-memory write.
**Sim/unit status:** checked in simulation.

| # | Step | Expected | Observed | Pass? |
|---|---|---|---|---|
| a | Normal operation across all modes; observe commanded torque (driver telemetry / logs). | All torques finite, |τ| within the arm's rated limits and ≤ 150 N·m. | | |
| b | (Optional, instrumented) lower `output_torque_limit_` to a value just above hold torque, rebuild, command an aggressive target. | Torque saturates at the ceiling; arm does not receive a runaway value. Restore the default afterwards. | | |
| c | Confirm no NaN/Inf ever reaches the driver during a full session. | Driver never reports a non-finite command. | | |

### 2.4 E-STOP — graceful ramp before kill

**What it does:** Space / toolbar E-STOP fans out to extensions, commands the
mode-0 torque ramp-down to every robot, waits a ramp window, locks the last pose,
then tears down processes.
**Sim/unit status:** unit-tested (`test_safety_estop.py`) — graceful stop issued
and ordered before the process kill.

| # | Step | Expected | Observed | Pass? |
|---|---|---|---|---|
| a | With the arm holding under control, press **Space**. | Torque **ramps down** to a safe hold (no abrupt drop / freeze); status bar shows "E-STOP ACTIVE". | | |
| b | Repeat with the toolbar **■ E-STOP** button. | Same graceful behaviour. | | |
| c | During a multi-robot session, trigger E-STOP. | Every robot ramps down, not just the active one. | | |
| d | Confirm processes (controller/driver) are torn down **after** the ramp. | No motion after teardown; arm safe. | | |

### 2.5 Runtime monitor — reactive graceful stop

**What it does:** the 10 Hz monitor checks the **actual** EE pose + joint angles
against the enabled limits (any mode) and commands a debounced graceful stop on a
real violation.
**Sim/unit status:** unit-tested (`test_safety_monitor.py`).

| # | Step | Expected | Observed | Pass? |
|---|---|---|---|---|
| a | With a zone enabled and the arm inside, **tighten** a limit so the current pose is now just outside. | Within ~1–2 control cycles the controller commands a graceful stop (debounced, once). | | |
| b | Check the log. | A single violation + stop entry (not a warn-only message, not a repeating spam). | | |

---

## 3. Controller shutdown

**What it does:** `fic_control` clears the shared-memory `alive` flag when it
shuts down, and the driver leaves torque mode when the flag drops.

| # | Step | Expected | Observed | Pass? |
|---|---|---|---|---|
| a | Stop the controller cleanly (SIGTERM) while the arm holds. | `alive` drops to 0; the driver leaves torque mode; the arm does not hold a frozen torque. | | |
| b | Kill the controller abruptly (SIGKILL). | The driver (or the inner-loop MCU, see `docs/mcu-inner-loop.md`) stops the arm safely. | | |

---

## 4. Regression suite (run before each hardware session)

The software-side nets have automated tests. Run them from
`fic_ws/src/fic_studio` and confirm green before going to hardware:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  test/test_safety_estop.py \
  test/test_safety_monitor.py \
  test/test_safety_zone_panel.py \
  test/test_cartesian_error_parse.py -q
# expected: all passed
```

Sim enforcement probes (controller-level, optional, need a sourced ROS shell):

- The Cartesian clamp, the joint-mode FK workspace guard and the output-torque
  guard can each be checked against a running `fic_simulation` controller.

---

## 5. Sign-off

| Net | Tester | Date | Result (pass/fail) | Notes |
|---|---|---|---|---|
| 2.1 Cartesian zone clamp | | | | |
| 2.2 Joint-mode FK workspace guard | | | | |
| 2.3 Output-torque guard | | | | |
| 2.4 E-STOP graceful ramp | | | | |
| 2.5 Runtime monitor stop | | | | |
| 3 Controller shutdown | | | | |

**Overall hardware sign-off:** ____________________  Date: __________
