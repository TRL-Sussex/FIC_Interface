# Changelog

All notable changes to **FIC Control Studio / FIC Interface** are recorded here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/).

### Versioning

Version numbers are given at release only. When Development is released, the release takes the
next number: a new **minor** version (x.Y.0) for features, a **patch** version (x.y.Z) for fixes
only, a new **major** version (X.0.0) for breaking or architectural changes. It is written
everywhere at once.
Between releases nothing is bumped: changes are recorded under **Unreleased**, and a development
build shows the last release and the commit it was built from (`5.11.0 · dev 85617ecb`). The
FIC-link protocol keep their own version numbers.

### Releases

Development happens in a separate repository. Each release of FIC Control
Studio is published here as one commit, with its entry below.

---

## [Unreleased]

## [5.11.0] — 2026-10-19 — Whole-arm safety zones, a safer operator interface, FIC library fixes

### Added
- **Whole-arm safety zones** (`fic_library/include/fic/safety_zones.h`): boxes (turned about the
  vertical), spheres, upright cylinders and outlines drawn on the floor and raised between two heights,
  each keep-in (the arm stays inside one of them) or keep-out (it stays out of every one). They are
  checked along the whole arm, from a chosen joint to the tool (each link a capsule, the tool a ball),
  not only at the tool point as the workspace box is. Both controller nodes enforce them in every mode:
  a virtual wall on each link past a boundary, joint targets that take the arm further in refused,
  Cartesian tool targets moved out of the zones; the arm's worst clearance is published on
  `robot{N}_safety/zone_state`, and Stop on violation stops a robot more than 30 mm into one.
- **The Safety page reworked**: the zones and the workspace box drawn live in 3D, with a toolbar on the
  view to place a box, sphere or cylinder with a click, draw an outline corner by corner, set a workspace
  limit where the tool is (Table here, Wall here, Set here) and delete the selected zone; handles to move,
  lift, resize and turn a zone; sliders next to every number, two-thumb sliders for the joint limits.
  What it applies is saved with the robot.
- **Warnings in Notices** when a robot touches a safety zone or a workspace face (within 5 mm) or goes
  past it: they drop down and go after six seconds, and stay in Notices.
- **Latched E-STOP**: a red banner says when and by whom; Start, motion, processes and teleoperation are
  refused until someone resets it. The E-STOP and its reset go into Notices and the audit log.
- **Notices**: failures and warnings, shown wherever the operator is and kept until Clear all.
- **Audit log** (`~/.fic_studio/audit.log`) and an **Audit** tab on the Log page.
- **Roles** on the Users page: Observer, Simulation operator, Operator, Admin.
- A **hardware band** under the toolbar on the hardware target, and **Resume** after a Pause.
- **Settings > Data**: experiments and bags can be recorded outside the checkout (`$FIC_EXPERIMENTS_DIR`).
- **Robot Config**: Cartesian admittance push limits per robot (`admittance_limits`) and the FIC law
  options (`fic_law`: restart on a stall, the planner's frequency cap and speed margin).

### Changed
- **The FIC law restarts a stalled convergence** in the tracking modes (1, 2, 5, 6; on by default,
  `fic_law.restart_on_stall`): a damped joint no longer comes to rest at half its largest error with no
  torque. Stepped moves in the Kinova simulation end 0.17° short instead of 2.64°.
- **The planner's `acc_max` caps its acceleration**, and joint planners never plan faster than the robot's
  max joint speed. The Tuning pages no longer offer the planner's stiffness and frequency, which do nothing.
- **Stop on violation** stops a robot clearly past a limit (30 mm past the workspace or into a safety zone,
  2° past a joint limit): the graceful stop, that robot only, once per episode. It did nothing before.
- E-STOP and the Studio's stops end only this Studio's processes (they used `pkill`, which also stopped
  other users' simulators on a shared PC). E-STOP no longer takes control away from the web UI's holder.
- Esc no longer fires E-STOP (Space still does); Home on hardware asks first; the Dashboard's safety lights
  turn grey without fresh data; self sign-up on the web remote is off by default; Session is under System
  in the side list.
- The FIC library and the workspace build with no compiler warnings; unused code was removed.

### Fixed
- **Joint Admittance moved the arm by itself** on joints whose range leaves out zero (the Franka's joints
  4 and 6).
- A profile with fewer `saturation_ratios` than bands read past the end of its list; two controllers in
  one process shared their joint limits and admittance state; Cartesian Admittance sent its moved goal to
  the Studio as zeros.
- KUKA iiwa 14: modes 3 and 4 had no planner, so a push barely moved the arm.
- **The 3D views turned a simulated arm over or made it jump** (in H-PMP, and when the mode was changed
  elsewhere than the Studio's selector): a simulated robot's joints now come from the simulator by name.
  The Dashboard's Cartesian error was read from the wrong place.
- The first-run wizard's test launch always failed.


---

## [5.10.0] — 2026-10-17 — Joint target speed; Studio window polish; Playground mesh formats; Help rewritten

### Added
- **Settings → Processes**, first on the Settings page: start the simulator without the controller, either FIC
  controller (simulation or hardware) or the robot driver on its own, and stop them. The Dashboard's Overview no
  longer has process buttons.
- MuJoCo Playground: **PLY, OFF, GLB and 3MF** files become objects too (turned into STL when dropped, with
  `trimesh`), files up to **250 MB**, and a mesh over MuJoCo's **200,000-triangle** limit is simplified to about
  150,000 triangles as it is added (a 2.2-million-triangle CAD export takes about 7 s). The docs list the limits.
- Files dropped **anywhere** on the Robot Configuration or MuJoCo Playground page are taken (before: only on the
  small drop box; a drop beside it did nothing).
- **Joint target speed** for joint impedance (modes 1 and 2): the target the joint law sees moves towards a
  command far from the arm (Home, a joint slider, a sequence step) at no more than this speed, instead of the
  law's full effort at once, which on the arm moved fast enough to hit the safety limits. Set on the Safety page
  ("Joint target speed", rad/s, 0 = off), sent as a 4th value of `robot{N}_safety/velocity_limits`, and for the
  arm in `robots_config.yaml` (`safety_zones.velocity.joint_target_speed`; the Kinova 0.5 rad/s). Both
  controller nodes use it (`fic/joint_target_slew.h`); the simulator starts with it off.
- `admittance_deadbands.cart_force_hardware` in `robots_config.yaml`: the Cartesian force deadband on
  the physical arm, when its force estimate needs a wider one than the simulator. `fic_hardware` uses
  it, and `fic_simulation` when it simulates the arm (`hardware_model` on); the ideal simulator keeps
  `cart_force`. Both nodes log the Cartesian admittance deadband they run with.

### Changed
- Studio window: opens maximised; pages use the window's width (up to about 2100 px) and Settings' sections flow
  into columns on a wide window; drop-down lists open in the Studio's theme; the Start button is green with white
  text (Stop stays red) and its small S badge is gone; menu-bar lists hang straight from the bar and stay on
  screen.
- The **Help** page is rewritten for the Studio window: its menu bar, top bar, control bar and pages, its
  single-key shortcuts (the Ctrl ones are listed as the classic window's), Processes, Safety, Troubleshooting and
  Glossary, all checked against the code.

### Fixed
- Studio window: the right-click menu showed black text on black, in the light theme too; on Settings, a page
  with its sections folded scrolled far past the last one; hovering Start in the light theme turned it all white,
  and the Playground's Start lost its label on hover (the window's Chromium 87 ignores `color-mix()`).
- MuJoCo Playground: the Robot and Scene boxes were blank when no robot could run or the scene had been edited.
- Kinova Gen3 on the arm: Cartesian HAP-FIC (mode 8) and H-PMP walked their goal up to 50 mm off the
  streamed path while tracking. Their task-space admittance is a mass and damper with no spring, and
  the arm's joint-torque force estimate reads friction and the arm's own motion as 4-8 N along the
  motion, so every peak over the 4 N deadband moved the goal further. The arm's Cartesian force deadband is now 10 N (`cart_force_hardware`). Pushes under 10 N no longer move the goal on the arm.

---

## [5.9.0] — 2026-10-16 — MuJoCo 3.15 everywhere; Playground objects from mesh files

### Changed
- **MuJoCo 3.15.0 for every simulator** (from 3.8.0 on Linux installs and 3.10.0 in the Docker image):
  `fic_install/install-deps.sh`, the Docker image and the macOS installer pin the same release, and
  the GPU simulator moves to **Newton 1.7.0**, the Newton release built for MuJoCo 3.15 (Newton pins
  its MuJoCo, so both simulators now run the same physics). Every robot model compiles to the same
  joints, actuators, masses and contacts as on 3.8 (only mesh bounding sizes move by about a micron).

### Added
- MuJoCo Playground: **your own objects from mesh files.** Drop an STL or OBJ file on the Objects panel
  (or choose one), in the Studio window: it is checked in MuJoCo, kept in
  `~/.fic_studio/playground/meshes/` and added to the scene at a free spot, its units guessed (a file
  more than 1 unit across is read as millimetres) and changeable with Edit (m, cm, mm, in). It collides
  as its convex hull (at most 64 vertices) and takes its inertia from that hull, with the mass you give
  (a Mass field for every object), so open or non-watertight meshes work too; its pose is the middle of
  its bounding box, so Pick aims at it. Up to 30 MB, 5 mm to 0.6 m across.
- MuJoCo Playground: the arm starts at its home pose (`q_home_rad`), for every robot it lists
  (`ExtensionHost.set_simulation_start_at_home`).

### Fixed
- Code that MuJoCo 3.11 and later broke, kept working on earlier releases too: `mj_fullM` takes the
  `MjData`, and a MuJoCo enum is
  no longer equal to the NumPy integer of a model array, which made the robot importer skip every
  joint when checking a MuJoCo model against its URDF (it compares plain ints now).
- The H-PMP toggle starts on, but a controller started after the Studio started with H-PMP off (the
  nodes miss the Studio's first `{rid}_hpmp_enable`), so pressing Cart ran the plain Cartesian mode
  until the toggle was changed. The Studio now sends the toggle as it shows just before every
  Cartesian mode (5-8) it sends: the Cart menu, the toolbar, the Robot menu, the other robot cards,
  the fleet page and the mode re-sent when the arm's controller starts. A Cartesian mode sent any other
  way (extensions, scripts) carries the last H-PMP setting sent to that robot (on by default), so an
  extension that turned H-PMP off keeps it off.
- Simulation: the Kinova Gen3's gripper starts open, as the real one rests (it started closed until
  the first gripper command). A scene names a servo's start setpoint as
  `<custom><numeric name="start_ctrl_<actuator>"/>`; the simulator holds it until the first command and
  after a reset.
- Simulation: free objects in a scene (the Playground's) made a fixed-base arm count as floating, so
  it skipped its start pose and warned that it might free-fall. Only a free joint that carries the
  actuated joints is now a robot's floating base.

---

## [5.8.0] — 2026-10-15 — MuJoCo Playground; Quick Actions for trajectories and sequences

### Added
- **MuJoCo Playground**, reworked as a sandbox for learning to program the robot (web page, Programming
  section): pick an arm and a ready-made scene (a table with cubes, a ball, a can; or your own objects),
  press Start, and the Studio's own simulator and FIC controller start with that scene, so every panel
  works on it. It lists where each object is (the simulator publishes their poses on
  `robot<N>_simulation/object_poses`), walks through exercises (move, record, play, trajectories, pick,
  script) and picks an object as the sequence `pick_<object>` (home, open, above it pointing down, down,
  close, lift) that can be stepped through and edited. Robots come from `robots_config.yaml` (an arm
  whose scene has the FIC torque motors; the others say why). It runs the ideal simulator unless
  "Simulate like the real robot" is on, and never starts on a robot running its real arm. The scene is
  built with MuJoCo's model editor and written to `~/.fic_studio/playground/` (it used to fail to load
  for every robot). Docs: fic_docs "MuJoCo Playground".
- **Quick Actions** for learning: New sequence, Step, Speed, and a trajectory template (a line, a
  circle or a figure-eight from where the end effector is, in a chosen plane: run it, or save one cycle
  as a sequence to compare); extensions add their own rows (`ExtensionHost.add_quick_action`), on the
  desktop and in the web dock.
- Sequences: the trajectory player (60 Hz targets, speed ramped up and down so it starts and stops at
  rest, 300 mm/s at most in simulation; on a real robot 50 mm/s, a confirmation and no mode change); a
  waypoint may carry its own `tolerance_mm` and `timeout_s`; on the Simulation target the robot is
  switched to a mode that follows each waypoint (Joint or Cartesian Impedance with Planner).
- Extension host: `set_simulation_scene` (a scene for the simulator), `set_simulation_env`,
  `stop_simulation`, `stop_fic_simulation`, `select_robot`, `control_mode`, `send_hpmp`,
  `refresh_profile`.
- Simulating like the real robot: `hardware_model: torque_sensor: output` reports the measured joint
  torque at the joint's output (less the joint's own friction, damping and rotor inertia, as a torque
  sensor after the gearbox reads it); the default, `motor`, is the actuator torque as before.
- `fic_install/WHAT-GETS-INSTALLED.md`, linked from the README's install section: every package,
  download, admin-rights step and file outside the repository that each install route (Linux,
  Linux by hand, macOS, Docker) adds, and how to remove them.

### Changed
- Kinova Gen3: modes 7 and 8 (Cartesian Admittance and Cartesian HAP-FIC) are stiffer near their
  goal (position law 520 N/m up to 2.5 cm, then the same force limits; mode 8's y axis 438 N/m), and
  their orientation law and mode 7's position law no longer rise past their first force limit (mode
  7 was 200 N/m against 13 N at 10 cm). Simulated like the real arm, they stopped 12-17 mm short of
  the goal after a push and crawled the rest of the way against joint friction; now about 3 mm.
- The Studio runs a robot simulated like the real one on its `default` config in simulation too (not its
  simulation-only `sim` config), so the tuning tried in simulation is the one the arm runs.

### Fixed
- Kinova Gen3 in simulation: the Robotiq gripper opens and closes like the real one. Its knuckles
  turned the wrong way, so "closed" was the real gripper's open width and "open" spread the fingers to
  124 mm; the commanded width now gives that opening within 1 mm (0 to 80 mm). The gripper now touches
  the floor, objects and the arm, and its finger pads touch each other.
- Extensions' mode changes (`ExtensionHost.set_mode`) did not reach the simulator: they sent only the
  mode's name, which the simulation node does not read; they now send the name and the number, as the
  mode buttons do.
- Quick Actions' sequence list on the desktop stayed empty (it was filled before the sequences loaded);
  it is read again whenever Quick Actions or Snapshot opens.
- Gripper waypoints of a sequence on the Simulation target drive the simulated gripper (they were only
  logged).
- On macOS `bash install.sh --dry-run` ran the full install (the macOS installer ignored its
  options); it now shows the plan and changes nothing.

---

## [5.7.0] — 2026-10-15 — H-PMP, on by default; mode names; Whole-Arm Experiments

### Changed
- The overlay that nests the Cartesian controller inside the joint HAP-FIC is called **H-PMP**
  everywhere a user sees it (the Studio's control bar and robot cards, the web interface, the Tuning
  page, the Help page, the docs and the logs). Scripts call `robot.mode.hpmp(on)`; `robot.mode.hapfic`
  is kept as an alias. The Studio publishes `robot{N}_hpmp_enable`; the nodes accept it and the old
  `robot{N}_hapfic_enable`. The tuning block in `config.xml` is named H-PMP (still `<mode id="100">`).
- H-PMP is **on by default** in the Studio's Cartesian modes (the toggle starts on and its state is
  latched for any controller that starts later); turning it off sends the Cartesian torques directly
  to the joints. The nodes' own default and the experiment runners (which set it explicitly) are
  unchanged.
- H-PMP also works on **mode 5**: its task layer then acts on the goal without its planner, as
  Cartesian impedance does.
- H-PMP's IK starts from its previous solution (both nodes), so the arm's redundant posture is held by
  the joint layer: a push on the elbow no longer drags the IK set-point with it, and the elbow no
  longer drifts on after a push. The measured posture is still used when the two are more than
  0.5 rad apart (after a mode switch).
- Mode names follow the Whole-Arm paper: mode 4 **Joint HAP-FIC**, mode 6 **Cartesian Impedance with
  Planner**, mode 8 **Cartesian HAP-FIC** (labels, Tuning page, Help, docs, `config.xml` names). The
  strings sent on `{rid}_mode_select` are unchanged. The control bar's mode menus name mode 4 and
  mode 8 HAP-FIC; the mode readout, the web interface's mode menu and the fleet cards show these
  names, and with the H-PMP toggle on they read *H-PMP* on mode 8 (the Whole-Arm experiments'
  controller) and *<mode> + H-PMP* on modes 5-7. The toggle's tooltips and the Help page say it nests
  the Cartesian mode in the joint HAP-FIC.
- Kinova Gen3: H-PMP's default tuning lets joint 4 yield (admittance InvM 0.1, as the contact
  profile) and gives its task-space admittance a lighter virtual mass (InvM 0.2, was 1): simulated
  like the real arm, H-PMP on mode 7 kept a small oscillation after a push with the previous one.
  H-PMP on mode 6 yields more to a push (19 mm for 15 N, was 8 mm).
- The simulator measures the H-PMP task deadband and velocity damper of a robot simulated like the
  real one (below) as the hardware node does.

### Fixed
- `install.sh` stopped at once on macOS with "PASS[@]: unbound variable" (macOS's bash 3.2
  takes an empty array as unbound). The README gives the macOS way to start the Studio from a
  terminal (`conda activate fic_ros`, `setup.zsh`). The Pendulum URDF was in the repository twice,
  as `Pendulum.urdf` and `pendulum.urdf`, which collide on a case-insensitive disk; `pendulum.urdf`
  is removed.

### Added
- **Simulating like the real robot.** A robot's `hardware_model` block in `robots_config.yaml`
  (joint friction, damping and rotor inertia, torque-sensor noise, the measured-torque filter, a
  command delay) makes the simulated arm behave like the real one: the simulator gives its joints
  these values and adds the noise, and `fic_simulation` measures torque as `fic_hardware` does (the
  robot's filter, H-PMP's push estimated from the joint torques instead of MuJoCo's contact force,
  the robot's H-PMP task deadband and velocity damper), so tunes made in simulation carry over.
  Robot Config → Hardware → *Simulate like the real robot* edits it; robots without it are
  simulated as before, and `FIC_HARDWARE_MODEL=off` turns it off for one run. The Kinova Gen3 has
  one (torque noise measured on the arm, friction estimated).
- `python3 -m fic_studio.hardware_model --env hardware --robot <name> --write` measures a robot's
  joint friction, damping and torque noise on the arm (one joint at a time through a slow sine)
  and writes its `hardware_model`. The import report of a new robot and its Overview (for robots
  with a hardware driver) point to it.

## [5.6.0] — 2026-10-14 — Teach Mode and Sequences in the web interface; desired wrench; Kinova simulation starts at home

### Added
- Teach Mode has its own web page, in the style of the Script Editor's: Mode and Robot, Start/Stop
  teaching, the live joints (°) and end-effector pose (mm, °), Record position (Space), auto-record,
  the gripper (width in mm, Open, Close, Send, Record gripper), the recorded waypoints with Go and
  Delete, and Save as sequence. It used to appear as a copy of its Qt widgets, whose Yes/No dialogs
  opened on the Studio's screen where a browser could not answer them.
- Sequences' web page is rebuilt the same way: sequence picker with New, Rename, Delete, Save file
  and Reload file; Play, Step, Stop, Loop and Send to; the waypoints with their hold times and Go,
  ↑, ↓, Edit and Delete; a form to type a waypoint (joint, Cartesian or gripper) or capture the
  robot's position; and the playback speed, tolerances and timeout.
- Both pages ask before anything moves the robot and again, in red, on the real robot; the Studio
  refuses a hardware move the page has not confirmed.
- Sequences and Teach Mode are in the public release.
- The Docker image is published as `ghcr.io/trl-sussex/fic_interface` (it was
  `ghcr.io/trl-sussex/fic-studio`); the launchers and `docker/compose.yaml` use the new name.

### Fixed
- Teach Mode showed the joints in radians labelled as degrees.
- Teach Mode's Go sent joint waypoints on the hardware command topic in simulation too.
- After a waypoint was deleted in Teach Mode, the Go and ✕ buttons of the rows below it addressed
  the wrong waypoints.
- Save as sequence opened the Sequences "Saved" dialog; it now saves without one.
- Sequences' joint tolerance was 2.9° while the page showed 3.0°.
- A Sequences hold time sent without a value raised an error instead of being refused.

### Added — desired wrench
- The task-space admittance of modes 7 and 8, and of HAP-FIC on them, is driven by the desired
  minus the measured wrench instead of the measured wrench alone, so it moves the goal until the
  measured wrench equals the desired one, within the mode's bound. The desired wrench is zero when
  the controller starts, which is the compliant behaviour of earlier versions.
- Set it with the **Wrench…** button in the Cart options (an editor with Apply and Zero), the
  *Desired wrench* row of the web interface's Cart panel, `robot.mode.desired_wrench(force, torque)`
  in the Script Editor, `set_desired_wrench()` on the extension host, or the
  `robot{N}_desired_wrench` topic (`geometry_msgs/Wrench`, N and N·m, in the frame and sign of the
  measured wrench). The topic is not latched, so a controller that restarts begins at zero again;
  a wrench with a non-finite value is refused.
- Library: `ficControl::setDesiredWrench()` / `desiredWrench()`.

### Changed — the Kinova Gen3 simulation starts at home
- A fixed-base arm whose robot config sets `sim_start_at_home: true` spawns at its `q_home_rad`
  and is held there until the first actuator command, so the controller starts at home instead of
  swinging the arm up from all zeros. The Kinova Gen3 sets it; other arms keep the all-zero start.
- Both simulators take the pose as the launch argument `initial_joint_positions:=q1,q2,...`.

### Changed — Kinova Gen3 simulation tuning
- The simulated arm moves faster and is easier to move by hand; hardware is unchanged. A large
  joint move (mode 2) settles within 0.01 rad in 2.7 s instead of 8.1 s, and a 29 cm Cartesian
  move (mode 6) within 3 mm in 9 s instead of stopping 8 cm short.
- The MuJoCo model's joint damping is 5 N·m·s/rad instead of 45 (the wall scenes keep 45).
- A robot's velocity safety block may set `sim_max_joint_vel`, used in simulation instead of
  `max_joint_vel`; the Kinova sets 3 rad/s (hardware keeps 1).
- The Kinova's `sim` profile adds mode 3 with the lower admittance damping of mode 4 on joints
  2-4, and modes 6-8 with the Cartesian planner at 0.25 m/s. The Config dropdown selects `sim` on
  the Simulation target and `default` on Hardware until a config is picked. Mode 4 is unchanged.

## [5.5.0] — 2026-10-14 — The Script Editor in the web interface

### Added — the Script Editor's own web page
- The web interface, which the Docker image opens, showed the Script Editor as a copy of its Qt
  widgets: the code could not be edited and the editor sat below a 72-line API list. It now has
  its own page: a full-width editor with line numbers, Python colouring and code-editor keys
  (Tab and Shift+Tab indent, Enter keeps the indent, Ctrl+S saves, Ctrl+Enter runs), the output
  under it, and Examples and API drawers. The API drawer searches the robot API and inserts a
  call at the cursor; Examples opens a copy of a starter script.
- Scripts are saved, run and stopped from any browser that holds control. A run goes to the
  Studio's own target, shown as "Runs on" above the editor, and asks first on Hardware. When a
  script fails, its line is marked and the line number in the output jumps to it.
- Unsaved edits are kept by the browser across page changes and reloads. The Classic page is
  unchanged; both pages show the same output.
- A script is ordinary Python with the Studio's rights: keep sign-in on when the web interface is
  open to a network (Script Editor docs page, "Who can run scripts").

### Changed
- Docker: the desktop session starts the Studio's web interface (`fic studio --web-ui`), as the
  desktop entry does; the Classic interface stays in the menu.
- Docker: `fic-selftest` checks the simulator window with the Franka FR3 when the image has no
  Spot model.
- The API reference lists properties such as `robot.state.joints` without brackets: they are
  read, not called.
- Docs: the sidebar hides pages that are not installed, and links to them become plain text.
- Web interface: extension pages no longer open with the "Which extensions show in my sidebar"
  box; Settings ▸ Extensions chooses the extensions.
- Docker: `--lan` (`-Lan` on Windows) turns sign-in on and self sign-up off. The launcher
  prints the address for phones and a password for the account `admin`, new every start
  (`FIC_WEB_ADMIN_PASSWORD`; `docker/compose.yaml` passes it through).
- Help: the Extensions section lists the extensions found in `Extensions/`, not a fixed list.
- `LICENSE` holds the MIT licence text only; the note on third-party licences is in the README,
  which now describes the project and has References (the FIC papers), License, Citing and
  Questions sections. `CITATION.cff` gives the citation. The ROS packages declare the MIT licence
  too (they said Apache-2.0).
- Package metadata: `fic_hardware`'s maintainer and description, and the `communication` and
  `menagerie` descriptions, are filled in.

