# Changelog

All notable changes to **FIC Control Studio / FIC Interface** are recorded here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and
the project uses [Semantic Versioning](https://semver.org/).

### Versioning

Every big change bumps the version: a new **minor** version (x.Y.0) for
significant features or fixes, a new **major** version (X.0.0) for breaking or
architectural changes. The number is updated everywhere together — `APP_VERSION`
in the Studio (`main_window_pro.py`) and the Dev Console (`main_window_dev.py`),
both splash screens (`entry_pro.py`, `entry_dev.py`), the Help page
(`help_mixin.py`), `fic_studio`'s `package.xml` and `setup.py`, `fic_hardware`'s
`package.xml`, and the docs badge, "What's new" box and Settings subtitle — with
an entry here.

### Releases

Development happens in a separate repository. Each release of FIC Control
Studio is published here as one commit, with its entry below.

---

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

