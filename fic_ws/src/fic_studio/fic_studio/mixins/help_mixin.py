"""
Help page mixin for FIC Control Studio v5.6.0.

Provides ``HelpMixin._build_help_page() -> QWidget`` which assembles a
searchable, theme-aware help page: a ``QLineEdit`` search bar and a
``QListWidget`` section navigator on the left, and a ``QTextBrowser``
content viewer on the right.

Content is driven by ``HELP_SECTIONS`` -- an ordered list of
``(key, title, html_or_callable)`` tuples covering 13 sections sourced
from the FIC Studio feature catalog.  The Extensions and Keyboard
Shortcuts sections are generated at open-time from live application data
so they never go stale.  The Scripting section is sourced from the live
``api_reference`` of ``RobotAPI`` so it stays current with the API.

All chrome colours are supplied by ``fic_studio.theme`` tokens; HTML
content colours are injected via a ``<style>`` block generated from
``theme.current()`` and re-injected on ``theme.signals.theme_changed``.
"""

import os
import re
import sys
import time
import webbrowser
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple, Union

from fic_studio import theme as _theme
from fic_studio.theme.design import rgba
from fic_studio.config import FIC_DOCS
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QMessageBox,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSplitter,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)


# ---------------------------------------------------------------------------
# HELP_SECTIONS -- 12 ordered sections
# ---------------------------------------------------------------------------

# Each entry: (key, display_title, html_string_or_callable)
# Callable entries are called at render time to produce live HTML.

HELP_SECTIONS: List[Tuple[str, str, Union[str, Callable]]] = []

# -- 1. Getting Started -------------------------------------------------------

_HTML_GETTING_STARTED = """
<h2>Getting Started</h2>
<p><b>FIC Control Studio v5.6.0</b> is the operator GUI for the Fractal
Impedance Control (FIC) framework. It connects to robots running the FIC
C++ controller over ROS 2 / DDS and lets you drive, tune, and monitor
up to four robot stations simultaneously.</p>

<h3>First-time setup</h3>
<ol>
  <li>The <b>First-Launch Wizard</b> runs automatically on the first start
      (re-launch via <em>Help &rarr; Run Setup Wizard</em>). It walks through:
      <ul>
        <li><b>Welcome / Robot</b> &ndash; choose your robot model.</li>
        <li><b>Workspace</b> &ndash; set the ROS 2 workspace path.</li>
        <li><b>Test Launch</b> &ndash; verify the simulator starts correctly.</li>
        <li><b>Finish</b> &ndash; mark setup as complete.</li>
      </ul>
  </li>
  <li>Open <b>Settings</b> (Ctrl+,) to configure appearance, network
      (ROS_DOMAIN_ID, partner IP), simulation backend, and shortcuts.</li>
  <li>Select a <b>Target</b> (Simulation / Hardware / VR) in the toolbar,
      then press <b>&#9654; Start</b> to launch the relevant processes.</li>
  <li>Open <b>&#9650; Joint</b> on the control bar, pick a control mode, set a
      target pose or joint angles, and press <b>Send</b>.</li>
</ol>

<h3>Key concepts</h3>
<ul>
  <li><b>FIC controller</b> &ndash; a real-time C++ loop reading
      <code>config.xml</code>; parameters edited in the Tuning panel
      (hot-reloaded without restarting).</li>
  <li><b>Station selector</b> &ndash; the toolbar [1&ndash;4] buttons switch
      which robot&rsquo;s topics the UI drives; each station publishes to
      <code>robot{N}_*</code> topics.</li>
  <li><b>Extensions</b> &ndash; optional feature packs enabled in
      Settings &rarr; Extensions; each adds a sidebar page.</li>
  <li><b>E-STOP</b> &ndash; Space bar, the &#11035; toolbar button, or the
      menu item immediately halts all motion and kills active processes.</li>
</ul>
"""

# -- 2. Window Layout ---------------------------------------------------------

_HTML_WINDOW_LAYOUT = """
<h2>Window Layout</h2>

<h3>Toolbar</h3>
<p>The toolbar (toggle with Ctrl+T) contains from left to right:</p>
<table>
  <tr><td><b>Target</b></td>
      <td>Simulation / Hardware / VR &mdash; selects which process set
          Start/Stop operates on.</td></tr>
  <tr><td><b>&#9654; Start / &#9632; Stop</b></td>
      <td>Launch or terminate the target&rsquo;s processes (simulator,
          FIC controller, driver).</td></tr>
  <tr><td><b>&#9654; EXP</b></td>
      <td>Start / stop an experiment recording session.</td></tr>
  <tr><td><b>&#9654; BAG</b></td>
      <td>Start / stop <code>ros2 bag record -a</code>.</td></tr>
  <tr><td><b>&#9654; PLT</b></td>
      <td>Launch the live plotter.</td></tr>
  <tr><td><b>Robot: [1&ndash;4]</b></td>
      <td>Station selector &mdash; which robot this machine drives.</td></tr>
  <tr><td><b>[robot type]</b></td>
      <td>Currently active robot model.</td></tr>
  <tr><td><b>Teleop</b></td>
      <td>External-control / slave lock checkbox.</td></tr>
  <tr><td><b>&#11035; E-STOP</b></td>
      <td>Immediate emergency stop (also Space / Escape).</td></tr>
</table>

<h3>Sidebar</h3>
<p>The left sidebar (toggle Ctrl+\\) gives one-click access to each page.
Extension pages appear below the built-in items, grouped by
<code>SIDEBAR_SECTION</code>.</p>

<h3>Status bar LEDs</h3>
<table>
  <tr><td><b>ROS2</b></td><td>ROS 2 node is active.</td></tr>
  <tr><td><b>FIC</b></td><td>FIC controller is running.</td></tr>
  <tr><td><b>SIM</b></td><td>MuJoCo / Newton simulator is running.</td></tr>
  <tr><td><b>BAGS</b></td><td>rosbag is currently recording.</td></tr>
  <tr><td><b>TELEOP</b></td>
      <td>Teleop session active; sub-label shows MASTER / SLAVE / IDLE.</td></tr>
  <tr><td><b>GAMEPAD</b></td><td>Gamepad controller is connected.</td></tr>
</table>
<p>LED colours: green = connected/ok, red = disconnected/error,
orange = warning, blue = active, grey = idle.
During E-STOP the status bar shows <b>E-STOP ACTIVE</b> in red.</p>

<h3>Log viewer</h3>
<p>A collapsible log strip at the bottom of the window (toggle Ctrl+L)
displays INFO / WARN / ERROR messages with a filter dropdown.</p>
"""

# -- 3. Pages -----------------------------------------------------------------

_HTML_PAGES = """
<h2>Pages</h2>

<h3>Dashboard (Ctrl+0 &mdash; index 0)</h3>
<p>Grouped numeric monitor refreshing at 20 Hz. Only the visible tab is
   refreshed; a channel with no live source reads <code>--</code>.</p>
<ul>
  <li><b>Overview</b> &ndash; <b>Base</b> (process state with start/stop,
      control mode, target, profile, telemetry rate); <b>Joints</b>
      (commanded, position, error, velocity, torque per joint);
      <b>End effector</b> (commanded / pose / error / velocity as XYZ plus
      the &theta;x &theta;y &theta;z extrinsic triple, and the tool wrench
      with a contact indicator); <b>Safety</b> (joint, velocity and
      force/torque limit rows).</li>
  <li><b>Detailed</b> &ndash; per-joint torque ceiling, percentage of torque
      limit and percentage of travel; plus which wrench source is feeding
      the readout.</li>
</ul>

<h3>Control bar (bottom of every page)</h3>
<p>Robot Control is no longer a page. Driving happens on the bar docked to the
   bottom of the window, present on every page:</p>
<ul>
  <li><b>&#9650; Joint</b> and <b>&#9650; Cart</b> &ndash; the eight control
      modes as two menus of four. Each menu owns its own modifiers
      (Admittance planner for joint; Postural / Elbow / Hapfic for Cartesian)
      and its config selector. Pressing a menu selects its base mode.</li>
  <li><b>Control surface</b> &ndash; joint commanded-vs-actual rows two per
      line, or EE position down the left and orientation down the right.
      Each field supports click-to-type and drag-to-scrub.</li>
  <li><b>&#9650; Gripper</b> &ndash; hidden for robots without one.</li>
  <li><b>&#8962; Home</b> &ndash; commands q_home_deg in joint modes, or
      default_ee_target in Cartesian modes without leaving Cartesian.</li>
  <li><b>&#9673; Snapshot</b> and <b>&#9650; Quick Actions</b> &ndash; record
      and play saved sequences.</li>
  <li><b>Live indicator</b> &ndash; mode, target and loop rate; green when
      telemetry is live, amber stale, grey offline, teal while playing.</li>
</ul>
<p>Target (Simulation / Hardware / VR), the robot selector and E-STOP live in
   the top toolbar. VR disables the bar's command surface &mdash; the headset
   is driving.</p>

<h3>Multi-Robot (Ctrl+1 &mdash; index 1)</h3>
<p>The &#65291; Add Robot cards for commanding up to four stations at once:</p>
<ul>
  <li>Per-card station, robot type, mode, config and joint targets.</li>
  <li>Robot selector and collapsible <b>Advanced</b> panel (null-space
      Postural / Elbow / Hapfic / Admittance-Planner overlays, config
      profile, presets, joysticks).</li>
  <li>Action bar: Home, Reset, Live-publish toggle, Send.</li>
</ul>

<h3>Tuning Panel (Ctrl+2 &mdash; index 2)</h3>
<ul>
  <li>Per-mode FIC parameter editing with a full-width interactive
      force-profile canvas.</li>
  <li>MetricField params and breakpoint strips (error_max / force_max).</li>
  <li>Collapsible Planner and Admittance sub-panels.</li>
  <li>Live tracking-error / effort / wrench strip.</li>
  <li>Changes save to <code>config.xml</code> and are hot-reloaded by the
      FIC controller without a restart.</li>
</ul>

<h3>Settings (Ctrl+, &mdash; index 9)</h3>
<p>Single-scroll card layout with sections:</p>
<ul>
  <li>Appearance (theme, font)</li>
  <li>Startup (default robot, auto-save)</li>
  <li>Network (ROS_DOMAIN_ID, partner IP, DDS mode) &mdash;
      <em>requires restart</em></li>
  <li>Simulation (MuJoCo vs Newton backend)</li>
  <li>Tolerances (joint deg, position mm, timeout s)</li>
  <li>Shortcuts (rebindable keys)</li>
  <li>Extensions (enable/disable) &mdash; <em>requires restart</em></li>
  <li>Gamepad (axis / button assignments)</li>
  <li>Developer (verbose logging, auto-save rosbag)</li>
</ul>

<h3>Robot Config (index 10)</h3>
<p>Add, edit, and delete robot definitions via a 7-tab form. Sync
definitions to the FIC <code>config.xml</code>.</p>

<h3>Topic Monitor (index 11)</h3>
<p>Live ROS 2 topic list with message types and values.</p>

<h3>Extension pages (index 3&ndash;8, 12+)</h3>
<p>One page per enabled extension; index allocated at load time.</p>
"""

# -- 4. Control Modes ---------------------------------------------------------

_HTML_CONTROL_MODES = """
<h2>Control Modes</h2>
<p>Select a mode from the control bar's Joint or Cart menu.
Joint modes show joint rows; Cartesian modes show EE pose fields.</p>

<table>
  <tr><td><b>1 &mdash; Joint Impedance</b></td>
      <td>Pure position stiffness K<sub>0</sub>(q<sub>des</sub>&minus;q<sub>act</sub>),
          no damping.</td></tr>
  <tr><td><b>2 &mdash; Joint Impedance + Planner</b></td>
      <td>Smooth trajectory with damping_ratio / vel_desired / acc_max.
          Recommended default for joint-space driving.</td></tr>
  <tr><td><b>3 &mdash; Joint Admittance</b></td>
      <td>Yields to external torque (InvM / Kv / flow / pose).</td></tr>
  <tr><td><b>4 &mdash; Joint Admittance Haptic</b></td>
      <td>Force-feedback teleop leader in joint space.</td></tr>
  <tr><td><b>5 &mdash; Cartesian Impedance</b></td>
      <td>End-effector spring, no damping.</td></tr>
  <tr><td><b>6 &mdash; Cartesian Planner</b></td>
      <td>Smooth EE trajectory (recommended for Cartesian driving).</td></tr>
  <tr><td><b>7 &mdash; Cartesian Admittance</b></td>
      <td>EE yields to the external wrench and stays where pushed (&plusmn;30&nbsp;cm / &plusmn;30&deg;).</td></tr>
  <tr><td><b>8 &mdash; Cartesian Admittance Haptic</b></td>
      <td>Cartesian teleop follower.</td></tr>
</table>

<h3>Modifiers (control bar &rarr; Options)</h3>
<table>
  <tr><td><b>Postural</b></td>
      <td>Null-space joint-target overlay.</td></tr>
  <tr><td><b>Elbow</b></td>
      <td>Null-space elbow-hold overlay.</td></tr>
  <tr><td><b>Hapfic</b></td>
      <td>Hybrid Cartesian+joint haptic overlay; engages on modes 6&ndash;8 (on 7/8 the
          mode's task-space admittance also moves its goal).</td></tr>
  <tr><td><b>Admittance Planner</b></td>
      <td>Planner inside admittance modes 3 and 4; on by default.</td></tr>
</table>
"""

# -- 6. Gamepad ---------------------------------------------------------------

_HTML_GAMEPAD = """
<h2>Gamepad (Kinova-Style)</h2>
<p>Built-in Xbox/compatible gamepad teleoperation following the Kinova paradigm.
Reads <code>/dev/input/jsN</code> directly &mdash; no extra ROS package required.</p>

<h3>Joint Mode (Left Stick Y + Right Stick Y)</h3>
<table>
  <tr><td><b>Left stick Y</b></td><td>Drives the selected joint (velocity control).</td></tr>
  <tr><td><b>Right stick Y</b></td><td>Drives the next joint (selected + 1).</td></tr>
  <tr><td><b>LB / RB</b></td><td>Cycle which joint is selected (J1 &rarr; J2 &rarr; &hellip; &rarr; Jn).</td></tr>
</table>

<h3>Cartesian Mode (All 4 Axes)</h3>
<table>
  <tr><td><b>Left stick X</b></td><td>Y translation (left / right).</td></tr>
  <tr><td><b>Left stick Y</b></td><td>Z translation (up / down).</td></tr>
  <tr><td><b>Right stick Y</b></td><td>X translation (forward / back).</td></tr>
  <tr><td><b>Right stick X</b></td><td>Yaw rotation.</td></tr>
</table>

<h3>Button Mapping (Defaults)</h3>
<table>
  <tr><td><b>Back</b></td><td>Toggle Joint / Cartesian space.</td></tr>
  <tr><td><b>Start</b></td><td>Cycle sub-mode (Impedance &rarr; Planner &rarr; Admittance &rarr; Haptic).</td></tr>
  <tr><td><b>LB / RB</b></td><td>Previous / Next joint (joint mode).</td></tr>
  <tr><td><b>LT / RT</b></td><td>Gripper close / open (proportional, velocity-based).</td></tr>
  <tr><td><b>A</b></td><td>Home position.</td></tr>
  <tr><td><b>B</b></td><td>E-STOP.</td></tr>
  <tr><td><b>X</b></td><td>Toggle Postural hold.</td></tr>
  <tr><td><b>Y</b></td><td>Toggle Elbow hold.</td></tr>
  <tr><td><b>D-pad Up/Down</b></td><td>Speed level (Slow / Medium / Fast).</td></tr>
  <tr><td><b>D-pad Left/Right</b></td><td>Cycle active robot (multi-robot).</td></tr>
  <tr><td><b>Guide</b></td><td>Reset target pose.</td></tr>
</table>

<h3>Speed Presets</h3>
<table>
  <tr><th>Level</th><th>Cart Step</th><th>Joint Vel</th></tr>
  <tr><td>Slow</td><td>0.5 mm / 0.25&deg;</td><td>10&deg;/s</td></tr>
  <tr><td><b>Medium</b> (default)</td><td>1.0 mm / 0.5&deg;</td><td>25&deg;/s</td></tr>
  <tr><td>Fast</td><td>2.0 mm / 1.0&deg;</td><td>50&deg;/s</td></tr>
</table>

<h3>Configuration</h3>
<p>Settings &rarr; Gamepad tab. Deadzone, axis inversion checkboxes, and full
button mapping editor. All buttons are remappable. The status bar shows the
current state (e.g. <code>Joint J1,J2 Med | Impedance</code>).</p>

<p>Gamepad publishes:</p>
<ul>
  <li><code>{rid}_desired_joint_angles</code>
      (<code>Float32MultiArray</code>, radians, all joints)</li>
  <li><code>command_ee_position_change_{rid}</code>
      (<code>geometry_msgs/Pose</code>, absolute pose)</li>
  <li><code>robot{N}_control_mode</code>
      (<code>Int32</code>) + <code>{rid}_mode_select</code>
      (<code>String</code>)</li>
  <li><code>{rid}_gripper_width</code>
      (<code>Float32</code>, metres)</li>
</ul>
"""

# -- 8. Processes -------------------------------------------------------------

_HTML_PROCESSES = """
<h2>Processes</h2>
<p>The toolbar <b>&#9654; Start</b> / <b>&#9632; Stop</b> and the Dashboard
Session card manage these processes:</p>

<table>
  <tr><td><b>MuJoCo Simulation</b></td>
      <td>Physics simulator (MuJoCo or Newton backend, set in
          Settings &rarr; Simulation).</td></tr>
  <tr><td><b>FIC Simulation Controller</b></td>
      <td>C++ FIC control loop connected to MuJoCo; reads
          <code>config.xml</code>.</td></tr>
  <tr><td><b>Hardware Driver</b></td>
      <td>Vendor driver (Kinova Kortex / Franka FCI / &hellip;).</td></tr>
  <tr><td><b>Hardware FIC Controller</b></td>
      <td>C++ FIC loop connected to real hardware.</td></tr>
  <tr><td><b>VR Relay</b></td>
      <td>DDS relay for VR teleop sessions.</td></tr>
  <tr><td><b>Rosbag</b></td>
      <td><code>ros2 bag record -a</code> &mdash; use &#9654; BAG in the
          toolbar or &#9654; EXP for named sessions.</td></tr>
  <tr><td><b>Plotter</b></td>
      <td>Live-plotting tool launched from &#9654; PLT in the toolbar.</td></tr>
</table>

<p>The Dashboard Session card provides per-process start/stop controls
for simulator, FIC controller, and driver. Status bar LEDs (FIC, SIM, BAGS)
reflect running state in real time.</p>
"""

# -- 9. Safety ----------------------------------------------------------------

_HTML_SAFETY = """
<h2>Safety</h2>

<h3>E-STOP</h3>
<p>Triggered by: <b>Space</b> key, <b>&#11035; E-STOP</b> toolbar button,
or the menu item.  Sequence (v3.9.9):</p>
<ol>
  <li>Fan out to every loaded extension (each halts its own activity).</li>
  <li><b>Command a graceful torque ramp-down</b> ("Stop Controller", mode&nbsp;0)
      to every robot and wait a short ramp window &mdash; the robot decays to a
      safe hold instead of freezing its last torque on an abrupt kill.</li>
  <li>Lock each robot at its last-known joint pose.</li>
  <li>Tear down all processes (controller, driver, FIC, simulator, rosbag).</li>
  <li>Status bar shows <b>E-STOP ACTIVE</b> in red.</li>
</ol>

<h3>Safety Zones</h3>
<p>Open the dedicated <b>&#9216;&nbsp;Safety</b> page in the sidebar (or edit in
<b>Robot Config &rarr; Safety Zones</b>).  The page shows an unmissable
<b>enabled/disabled banner</b>, per-axis Cartesian workspace bounds and
per-joint angle bounds, mode-aware emphasis (joint vs Cartesian), and a
<b>live readout</b> of the current position against each limit (green in&#8209;zone,
red past it).</p>
<ul>
  <li><b>Cartesian workspace box</b> &mdash; X / Y / Z min/max (metres).</li>
  <li><b>Joint angle bounds</b> &mdash; per-joint min/max (degrees).</li>
</ul>
<p>Enforcement (verified in simulation):</p>
<ul>
  <li>The controller <b>clamps commanded targets</b> to the enabled limits
      (joint and Cartesian) before acting on them.</li>
  <li><b>Joint mode &rarr; EE workspace:</b> each candidate joint target is run
      through forward kinematics and <b>rejected</b> if the resulting EE would
      leave the enabled Cartesian box &mdash; so the EE stays in its workspace even
      when commanding in joint space.</li>
  <li>The 10&nbsp;Hz monitor <b>commands a graceful stop</b> (debounced) when the
      robot is actually past a limit &mdash; not just a warning.</li>
</ul>
<p>Limit topics (per station N, <code>Float64MultiArray</code>):</p>
<ul>
  <li><code>robot{N}_safety/cartesian_limits</code></li>
  <li><code>robot{N}_safety/joint_limits</code></li>
  <li><code>robot{N}_safety/velocity_limits</code></li>
</ul>

<h3>Output-torque guard</h3>
<p>Before publishing, commanded torques are forced finite (NaN/Inf&nbsp;&rarr;&nbsp;0)
and clamped to a safe per-joint ceiling &mdash; a backstop against control
output that has gone non-finite or runaway.</p>

<h3>Teleoperation</h3>
<p>The <b>Teleop</b> checkbox puts the station into external-control / slave
mode.  Bilateral force reflection to the operator's haptic device is
<b>gain-scaled, NaN-guarded, and saturated</b> (force / torque ceilings) so a
spiking F/T reading cannot be amplified unbounded onto the operator.</p>
"""

# -- 10. Troubleshooting ------------------------------------------------------

_HTML_TROUBLESHOOTING = """
<h2>Troubleshooting</h2>

<h3>Robot does not respond to Send</h3>
<ul>
  <li>Check the <b>FIC</b> LED &mdash; the controller must be running.</li>
  <li>Verify <b>ROS_DOMAIN_ID</b> matches the controller
      (Settings &rarr; Network, then restart).</li>
  <li>Confirm the correct station number [1&ndash;4] is selected in the
      toolbar.</li>
  <li>In Hardware mode, check the <b>ROS2</b> and driver LEDs.</li>
</ul>

<h3>Simulator will not start</h3>
<ul>
  <li>Run the First-Launch Wizard (Help &rarr; Run Setup Wizard) to
      verify workspace paths.</li>
  <li>Try switching the backend in Settings &rarr; Simulation (MuJoCo vs
      Newton).</li>
  <li>Inspect the log viewer (Ctrl+L) for startup errors.</li>
</ul>

<h3>Gamepad not detected</h3>
<ul>
  <li>Plug the controller in before starting the Studio.</li>
  <li>Check <code>/dev/input/jsN</code> exists
      (<code>ls /dev/input/js*</code>).</li>
  <li>The <b>GAMEPAD</b> LED turns green when the pad is enumerated.</li>
</ul>

<h3>Extension page is missing</h3>
<ul>
  <li>Enable the extension in Settings &rarr; Extensions, then
      <b>restart the Studio</b> (extensions load at startup).</li>
  <li>Check the log viewer for import errors.</li>
</ul>

<h3>Tuning changes are not applied</h3>
<ul>
  <li>Press <b>Save</b> in the Tuning panel to write
      <code>config.xml</code>.</li>
  <li>The FIC controller watches <code>config.xml</code> and hot-reloads;
      if it does not pick up the change, restart the controller.</li>
</ul>

<h3>Theme looks wrong after restart</h3>
<ul>
  <li>Open Settings &rarr; Appearance, pick the desired theme, and click
      <b>SAVE SETTINGS</b>.</li>
</ul>
"""

# -- 11. Glossary -------------------------------------------------------------

_HTML_GLOSSARY = """
<h2>Glossary</h2>
<table>
  <tr><td><b>FIC</b></td>
      <td>Fractal Impedance Control &mdash; the proprietary real-time
          controller at the core of the framework.</td></tr>
  <tr><td><b>EE</b></td>
      <td>End-Effector &mdash; the robot&rsquo;s tool / flange point.</td></tr>
  <tr><td><b>DOF</b></td>
      <td>Degrees of Freedom.</td></tr>
  <tr><td><b>DDS</b></td>
      <td>Data Distribution Service &mdash; the ROS 2 middleware layer
          (CycloneDDS by default).</td></tr>
  <tr><td><b>ROS 2</b></td>
      <td>Robot Operating System 2 &mdash; the inter-process communication
          layer used by the Studio.</td></tr>
  <tr><td><b>config.xml</b></td>
      <td>The FIC controller&rsquo;s parameter file; edited in the Tuning
          panel and hot-reloaded without a restart.</td></tr>
  <tr><td><b>Admittance</b></td>
      <td>A compliance mode where the robot yields to applied forces /
          torques (modes 3, 4, 7, 8).</td></tr>
  <tr><td><b>Impedance</b></td>
      <td>A stiffness mode where the robot resists deviation from the
          commanded position (modes 1, 2, 5, 6).</td></tr>
  <tr><td><b>Hapfic</b></td>
      <td>Hybrid Cartesian+joint haptic overlay that engages on modes 6&ndash;8: on 6 the
          arm returns to the commanded goal after a push, on 7 the goal stays where it was
          pushed, on 8 it gives way up to 5&nbsp;cm; configured in the control bar's Options row.</td></tr>
  <tr><td><b>Null-space</b></td>
      <td>The set of joint motions that do not move the EE; used by the
          Postural and Elbow overlays.</td></tr>
  <tr><td><b>Station</b></td>
      <td>A numbered robot slot (1&ndash;4); each station maps to a
          <code>robot{N}_*</code> topic namespace.</td></tr>
  <tr><td><b>Extension</b></td>
      <td>An optional feature pack living in <code>Extensions/</code>;
          enabled in Settings &rarr; Extensions and loaded at startup.</td></tr>
  <tr><td><b>MASTER / SLAVE</b></td>
      <td>Teleop roles: MASTER sends force-feedback commands; SLAVE
          receives and follows them.</td></tr>
  <tr><td><b>MuJoCo</b></td>
      <td>Physics simulator backend (CPU); alternative to Newton (GPU).</td></tr>
  <tr><td><b>Newton</b></td>
      <td>NVIDIA Warp-based GPU physics backend (optional install).</td></tr>
</table>
"""

# -- 12. About ----------------------------------------------------------------

_HTML_ABOUT = """
<h2>About FIC Control Studio</h2>
<p><b>FIC Control Studio v5.6.0</b></p>
<p>The operator GUI for the Fractal Impedance Control robot framework.</p>
<ul>
  <li><b>Package:</b> <code>fic_studio</code></li>
  <li><b>Settings file:</b> <code>~/.robot_selector_gui_settings.yaml</code></li>
  <li><b>Tech stack:</b> Python 3, PyQt5, ROS 2, MuJoCo / Newton</li>
</ul>
<p>Extensions, themes, and the UI architecture follow the v3.0.0 extension
system: optional features live in <code>Extensions/&lt;name&gt;/</code> and
are loaded via the extension host at startup.</p>
<p>For documentation, source, and issue tracking, use
<em>Help &rarr; Open Documentation</em>.</p>
"""


def _scripting_html() -> str:
    """
    Generate the Scripting section HTML from ``api_reference``.

    Imports ``RobotAPI`` and ``api_reference`` from the Script Editor
    extension at call-time so the section is always current.  Falls back
    gracefully if the extension is not installed or the import fails,
    returning a minimal static description instead.

    The returned HTML is **not** wrapped in the themed ``<style>`` block;
    ``HelpMixin._help_render_section`` passes it through
    ``_help_themed_html`` before display.
    """
    rows = ['<h2>Scripting</h2>']
    rows.append(
        '<p>The <b>Script Editor</b> extension (Ctrl+6) lets you write and run '
        'Python scripts that control the robot in real time.  Every script '
        'receives a <code>robot</code> object with grouped sub-namespaces '
        'plus flat alias functions for backward compatibility.</p>'
    )

    # ── robot facade overview ────────────────────────────────────────────
    rows.append('<h3>robot object</h3>')
    rows.append(
        '<p>The <code>robot</code> variable is a <code>RobotAPI</code> instance '
        'with the following sub-namespaces:</p>'
        '<ul>'
        '<li><code>robot.state</code> &mdash; read joint positions, EE pose, '
        'velocities, effort, wrench (degrees; thread-safe).</li>'
        '<li><code>robot.motion</code> &mdash; move_joints, move_cartesian, '
        'home, set_gripper.</li>'
        '<li><code>robot.mode</code> &mdash; set(), postural(), elbow(), '
        'hapfic(), admittance_planner().</li>'
        '<li><code>robot.process</code> &mdash; start/stop sim, FIC, driver, '
        'VR, bags, plotter.</li>'
        '<li><code>robot.target</code> &mdash; current target and set_target().</li>'
        '<li><code>robot.tune</code> &mdash; param_update(), save().</li>'
        '<li><code>robot.safety</code> &mdash; estop(), broadcast_stop(), pause().</li>'
        '<li><code>robot.topics</code> &mdash; subscribe(), get(), '
        'list_topics(), unsubscribe().</li>'
        '</ul>'
    )

    # ── flat aliases ─────────────────────────────────────────────────────
    rows.append('<h3>Flat aliases (backward compatible)</h3>')
    rows.append(
        '<p>These names are injected directly as globals so old scripts '
        'continue to work:</p>'
        '<table>'
        '<tr><td><code>move_joints(angles_deg, wait=False, timeout=10.0)</code>'
        '</td><td>&rarr; robot.motion.move_joints</td></tr>'
        '<tr><td><code>move_cartesian(x, y, z, rx, ry, rz)</code>'
        '</td><td>&rarr; robot.motion.move_cartesian</td></tr>'
        '<tr><td><code>home()</code>'
        '</td><td>&rarr; robot.motion.home</td></tr>'
        '<tr><td><code>set_gripper(width_m)</code>'
        '</td><td>&rarr; robot.motion.set_gripper</td></tr>'
        '<tr><td><code>get_joint_positions()</code>'
        '</td><td>&rarr; robot.state.joints (degrees)</td></tr>'
        '<tr><td><code>get_ee_position()</code>'
        '</td><td>&rarr; robot.state.ee_pose[:3]</td></tr>'
        '<tr><td><code>set_mode(mode)</code>'
        '</td><td>&rarr; robot.mode.publish_mode</td></tr>'
        '<tr><td><code>wait(seconds)</code>'
        '</td><td>&rarr; robot.wait (honours Stop)</td></tr>'
        '<tr><td><code>is_stopped()</code>'
        '</td><td>&rarr; True when Stop is requested</td></tr>'
        '<tr><td><code>stop()</code>'
        '</td><td>&rarr; robot.safety.estop</td></tr>'
        '</table>'
    )

    # ── two short examples ───────────────────────────────────────────────
    rows.append('<h3>Example: Read state</h3>')
    rows.append(
        '<pre>'
        'joints = robot.state.joints    # list[float], degrees\n'
        'print(f"J1: {joints[0]:.1f} deg")\n'
        'ee = robot.state.ee_pose\n'
        'if ee:\n'
        '    print(f"EE: x={ee[0]:.3f}  y={ee[1]:.3f}  z={ee[2]:.3f} m")'
        '</pre>'
    )

    rows.append('<h3>Example: Joint trajectory (blocking)</h3>')
    rows.append(
        '<pre>'
        'WAYPOINTS = [\n'
        '    [0.0, -45.0, 0.0, -90.0, 0.0, 45.0, 0.0],\n'
        '    [30.0, -30.0, 20.0, -60.0, 0.0, 60.0, 0.0],\n'
        ']\n'
        'for wp in WAYPOINTS:\n'
        '    if is_stopped():\n'
        '        break\n'
        '    robot.motion.move_joints(wp, wait=True, timeout=15.0)\n'
        '    robot.wait(0.3)'
        '</pre>'
    )

    # ── live api_reference table ─────────────────────────────────────────
    rows.append('<h3>Full API reference</h3>')
    try:
        import sys as _sys
        import pathlib as _pathlib
        _ext_root = str(
            _pathlib.Path(__file__).resolve().parent.parent.parent.parent.parent.parent
        )
        if _ext_root not in _sys.path:
            _sys.path.insert(0, _ext_root)
        from Extensions.script_editor.robot_api import (  # noqa: PLC0415
            RobotAPI as _RobotAPI,
            api_reference as _api_reference,
        )

        class _StubNode:
            """Minimal node stub for api_reference generation."""

            _MODE_NAMES = [
                'Joint Impedance', 'Joint Planner', 'Joint Admittance',
                'Joint Admittance Haptic', 'Cartesian Impedance',
                'Cartesian Planner', 'Cartesian Admittance',
                'Cartesian Admittance Haptic',
            ]

            def publish_mode(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_postural_enable(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_elbow_enable(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_hapfic_enable(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_admittance_planner(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_param_update(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_joint_angles(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_real_joint_angles(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_position(self, *_a, **_kw):
                """No-op publish stub."""

            def publish_gripper_width(self, *_a, **_kw):
                """No-op publish stub."""

            def get_topic_names_and_types(self):
                """Return empty topic list stub."""
                return []

            active_profile_publisher_ = None

        class _StubHost:
            """Minimal host stub for api_reference generation."""

            _mw = None

            def joint_positions_for(self, _rid):
                """Return empty joint list stub."""
                return []

            def joint_velocity_for(self, _rid):
                """Return empty velocity list stub."""
                return []

            def joint_effort_for(self, _rid):
                """Return empty effort list stub."""
                return []

            def joint_commanded_for(self, _rid):
                """Return empty commanded list stub."""
                return []

            def ee_pose_for(self, _rid):
                """Return None EE pose stub."""
                return None

            def wrench_for(self, _rid):
                """Return None wrench stub."""
                return None

            def robot_target(self, _rid):
                """Return simulation target stub."""
                return 'simulation'

            def broadcast_stop(self):
                """No-op broadcast stop stub."""

        import threading as _threading

        _stop_ev = _threading.Event()
        _api = _RobotAPI(
            host=_StubHost(),
            node=_StubNode(),
            rid='robot1',
            target='simulation',
            marshal=lambda fn, **_kw: None,
            is_stopped=_stop_ev.is_set,
        )
        entries = _api_reference(_api)
        if entries:
            rows.append(
                '<p>Auto-generated from <code>api_reference()</code> — '
                'always current with the installed extension.</p>'
                '<table>'
            )
            current_ns = None
            for entry in entries:
                ns = entry['name'].rsplit('.', 1)[0] if '.' in entry['name'] else ''
                if ns != current_ns:
                    current_ns = ns
                    rows.append(
                        f'<tr><td colspan="3"><b>{ns}</b></td></tr>'
                    )
                name = entry['name']
                sig = entry.get('signature', '')
                doc = entry.get('doc', '')
                rows.append(
                    f'<tr>'
                    f'<td><code>{name}{sig}</code></td>'
                    f'<td>{doc}</td>'
                    f'</tr>'
                )
            rows.append('</table>')
        else:
            rows.append('<p><em>No entries returned by api_reference().</em></p>')
    except Exception as _exc:  # noqa: BLE001
        rows.append(
            '<p><em>Scripting API reference could not be loaded '
            f'({type(_exc).__name__}: {_exc}).  '
            'See <code>Extensions/script_editor/robot_api.py</code> '
            'for the full reference.</em></p>'
        )

    rows.append(
        '<p>Full reference: '
        '<code>docs/scripting/API.md</code></p>'
    )
    return '\n'.join(rows)


def _build_sections() -> List[Tuple[str, str, Union[str, Callable]]]:
    """
    Return the ordered HELP_SECTIONS list.

    Callable entries are resolved at render time so live data (loaded
    extension instances, current shortcut bindings) is always reflected.
    The list is built once at import time; the callables are lightweight
    closures invoked each time the user opens that section.
    """
    # Extensions and Shortcuts content slots are None; HelpMixin resolves
    # them at render time via _help_extensions_html / _help_shortcuts_html.
    return [
        ('getting_started', 'Getting Started', _HTML_GETTING_STARTED),
        ('window_layout', 'Window Layout', _HTML_WINDOW_LAYOUT),
        ('pages', 'Pages', _HTML_PAGES),
        ('control_modes', 'Control Modes', _HTML_CONTROL_MODES),
        ('extensions', 'Extensions', None),
        ('gamepad', 'Gamepad', _HTML_GAMEPAD),
        ('keyboard_shortcuts', 'Keyboard Shortcuts', None),
        ('scripting', 'Scripting', _scripting_html),
        ('processes', 'Processes', _HTML_PROCESSES),
        ('safety', 'Safety', _HTML_SAFETY),
        ('troubleshooting', 'Troubleshooting', _HTML_TROUBLESHOOTING),
        ('glossary', 'Glossary', _HTML_GLOSSARY),
        ('about', 'About', _HTML_ABOUT),
    ]


HELP_SECTIONS = _build_sections()

# Keep a set of all expected keys for validation / testing.
_EXPECTED_KEYS = {k for k, _, __ in HELP_SECTIONS}


# ---------------------------------------------------------------------------
# HelpMixin
# ---------------------------------------------------------------------------


def docs_directory() -> Path:
    """Return the bundled documentation directory.

    Uses the project's own repo-root resolver rather than a fixed parent
    walk. The Studio runs from a colcon *copy* install on macOS/RoboStack,
    where --symlink-install is unsupported, and a counted walk lands inside
    fic_ws/install/ — which is exactly what silently broke this button.

    FIC_ROOT is read here rather than only at import, so the override this
    function's own error message tells you about actually takes effect.
    """
    root = os.environ.get("FIC_ROOT")
    if root:
        return Path(root).expanduser().resolve() / "fic_docs"
    return Path(FIC_DOCS)


class HelpMixin:
    """
    Mixin that contributes the searchable Help page to the main window.

    Intended to be mixed into the ``ProfessionalRobotGUI`` class alongside
    other mixins.  Requires no ``super().__init__`` call of its own.

    Public API
    ----------
    _build_help_page() -> QWidget
        Build and return the fully assembled help page.

    Internal helpers
    ----------------
    _help_extensions_html() -> str
    _help_shortcuts_html() -> str
    _help_themed_html(body: str) -> str
    """

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def _build_help_page(self) -> QWidget:
        """
        Build and return the help page widget.

        Layout: QHBoxLayout with a left QVBoxLayout (search QLineEdit +
        section QListWidget) and a QTextBrowser content view, separated
        by a QSplitter.
        """
        root = QWidget()
        root.setObjectName('HelpPage')
        outer = QHBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setHandleWidth(1)
        outer.addWidget(splitter)

        # -- left panel --------------------------------------------------
        left_widget = QWidget()
        left_widget.setObjectName('HelpNavPanel')
        _theme.bind(
            left_widget,
            lambda t: (
                'QWidget#HelpNavPanel {'
                f'background:{t["bg_raised"]};'
                f'border-right:1px solid {t["border_subtle"]};'
                '}'
            ),
        )
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(8, 12, 8, 8)
        left_layout.setSpacing(6)

        # Search field
        self._help_search = QLineEdit()
        self._help_search.setObjectName('HelpSearch')
        self._help_search.setPlaceholderText('Search help...')
        self._help_search.setClearButtonEnabled(True)
        _theme.bind(
            self._help_search,
            lambda t: (
                'QLineEdit#HelpSearch {'
                f'background:{t["bg_overlay"]};'
                f'color:{t["text_primary"]};'
                f'border:1px solid {t["border_medium"]};'
                'border-radius:4px;'
                'padding:4px 8px;'
                '}'
                'QLineEdit#HelpSearch:focus {'
                f'border:1px solid {t["accent_blue"]};'
                '}'
            ),
        )
        left_layout.addWidget(self._help_search)

        # Section nav list
        self._help_nav = QListWidget()
        self._help_nav.setObjectName('HelpNav')
        self._help_nav.setFrameShape(self._help_nav.NoFrame)
        _theme.bind(
            self._help_nav,
            lambda t: (
                'QListWidget#HelpNav {'
                f'background:{t["bg_raised"]};'
                f'color:{t["text_primary"]};'
                'border:none;'
                'font-size:13px;'
                '}'
                'QListWidget#HelpNav::item {'
                'padding:6px 8px;'
                'border-radius:3px;'
                '}'
                'QListWidget#HelpNav::item:hover {'
                f'background:{t["bg_hover"]};'
                '}'
                'QListWidget#HelpNav::item:selected {'
                f'background:{t["bg_selected"]};'
                f'color:{t["text_primary"]};'
                '}'
            ),
        )

        # Populate nav
        self._help_section_items: List[Tuple[str, str, QListWidgetItem]] = []
        for key, title, _ in HELP_SECTIONS:
            item = QListWidgetItem(title)
            item.setData(Qt.UserRole, key)
            self._help_nav.addItem(item)
            self._help_section_items.append((key, title, item))

        # Build plain-text search corpus for static (non-callable) sections
        # so that _on_help_search_changed can match against body content too.
        # Callable sections (extensions, keyboard_shortcuts) are excluded;
        # they remain visible whenever their title does not match the query.
        _tag_re = re.compile(r'<[^>]+>')
        self._help_search_corpus: Dict[str, str] = {}
        for key, _title, content in HELP_SECTIONS:
            if isinstance(content, str):
                self._help_search_corpus[key] = _tag_re.sub(' ', content).lower()

        left_layout.addWidget(self._help_nav)

        # "Open Full Documentation" button
        self._help_docs_btn = QPushButton("Open Full Documentation")
        self._help_docs_btn.setObjectName("HelpDocsBtn")
        self._help_docs_btn.setCursor(Qt.PointingHandCursor)
        _theme.bind(
            self._help_docs_btn,
            lambda t: (
                "QPushButton#HelpDocsBtn {"
                "background:transparent;"
                f"color:{t['accent_blue']};"
                f"border:1px solid {t['accent_blue']};"
                "border-radius:5px;"
                "padding:8px 12px;"
                "font-weight:500;"
                "font-size:12px;"
                "}"
                "QPushButton#HelpDocsBtn:hover {"
                f"background:{rgba(t['accent_blue'], 0.10)};"
                "}"
            ),
        )
        self._help_docs_btn.clicked.connect(self._help_open_docs_website)
        left_layout.addWidget(self._help_docs_btn)

        # -- right panel: QTextBrowser ------------------------------------
        self._help_browser = QTextBrowser()
        self._help_browser.setObjectName('HelpBrowser')
        self._help_browser.setReadOnly(True)
        self._help_browser.setOpenLinks(False)
        _theme.bind(
            self._help_browser,
            lambda t: (
                'QTextBrowser#HelpBrowser {'
                f'background:{t["bg_base"]};'
                f'color:{t["text_primary"]};'
                'border:none;'
                'font-size:13px;'
                'padding:16px;'
                '}'
            ),
        )

        splitter.addWidget(left_widget)
        splitter.addWidget(self._help_browser)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([220, 780])

        # -- connections -------------------------------------------------
        self._help_nav.currentItemChanged.connect(self._on_help_nav_changed)
        self._help_search.textChanged.connect(self._on_help_search_changed)

        # Re-render current section on theme change.
        _theme.signals.theme_changed.connect(self._on_help_theme_changed)

        # Select the first section by default.
        if self._help_nav.count():
            self._help_nav.setCurrentRow(0)

        return root

    # ------------------------------------------------------------------
    # Navigation / rendering
    # ------------------------------------------------------------------

    def _on_help_nav_changed(
        self,
        current: Optional[QListWidgetItem],
        _previous: Optional[QListWidgetItem],
    ) -> None:
        """Render the selected section into the QTextBrowser."""
        if current is None:
            return
        key = current.data(Qt.UserRole)
        self._help_render_section(key)

    #: Port the bundled documentation is served on.
    DOCS_PORT = 8742

    def serve_docs(self, page_id: str = "") -> bool:
        """Serve fic_docs over localhost and open it, at ``page_id`` if given.

        HTTP rather than file://: the docs are a single page that fetches
        pages/*.html, and browsers refuse those requests from a file://
        origin — opening the file directly renders "Could not load".

        Returns True if a browser was opened.
        """
        docs_dir = docs_directory()
        index = docs_dir / "index.html"
        if not index.exists():
            self.log_message("WARN", f"Documentation not found at {index}", "Help")
            QMessageBox.warning(
                self, "Documentation not found",
                f"Could not find the documentation at:\n{index}\n\n"
                "Set FIC_ROOT to the FIC_Interface repo root if it lives "
                "somewhere unusual.")
            return False

        import subprocess
        port = self.DOCS_PORT
        proc = getattr(self, "_help_docs_server", None)
        if proc is not None and proc.poll() is not None:
            proc = None                     # died since last time
        if proc is None:
            try:
                proc = subprocess.Popen(
                    [sys.executable, "-m", "http.server", str(port),
                     "--directory", str(docs_dir)],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
            except OSError as exc:
                self.log_message("ERROR", f"Could not serve docs: {exc}", "Help")
                QMessageBox.warning(
                    self, "Could not open documentation",
                    f"Failed to start the documentation server:\n{exc}")
                return False
            time.sleep(0.4)
            if proc.poll() is not None:     # exited at once — usually the port
                detail = (proc.stderr.read().decode(errors="replace").strip()
                          if proc.stderr else "")
                self.log_message("ERROR", f"Docs server exited: {detail}", "Help")
                QMessageBox.warning(
                    self, "Could not open documentation",
                    f"The documentation server stopped immediately.\n\n"
                    f"Port {port} may already be in use.\n\n{detail[:300]}")
                return False
            self._help_docs_server = proc

        anchor = ("#" + page_id) if page_id else ""
        webbrowser.open(f"http://localhost:{port}/index.html{anchor}")
        self.log_message("INFO", f"Documentation served on port {port}", "Help")
        return True

    def _help_open_docs_website(self) -> None:
        """Open the bundled documentation from the Help page button."""
        self.serve_docs()

    def _help_render_section(self, key: str) -> None:
        """Look up ``key`` in HELP_SECTIONS and render its HTML."""
        body: Optional[str] = None
        for sec_key, _, content in HELP_SECTIONS:
            if sec_key == key:
                if callable(content):
                    body = content()
                elif content is None:
                    # Callable sections where content slot is None are
                    # resolved by the dedicated helper methods.
                    if key == 'extensions':
                        body = self._help_extensions_html()
                    elif key == 'keyboard_shortcuts':
                        body = self._help_shortcuts_html()
                    else:
                        body = '<p>No content available.</p>'
                else:
                    body = content
                break
        if body is None:
            body = '<p>Section not found.</p>'
        self._help_browser.setHtml(self._help_themed_html(body))

    def _on_help_theme_changed(self, _name: str) -> None:
        """Re-render the current section so HTML colours update."""
        current = getattr(self, '_help_nav', None)
        if current is None:
            return
        item = current.currentItem()
        if item is not None:
            key = item.data(Qt.UserRole)
            self._help_render_section(key)

    # ------------------------------------------------------------------
    # Search
    # ------------------------------------------------------------------

    def _on_help_search_changed(self, text: str) -> None:
        """
        Filter the nav list by ``text`` (case-insensitive title or body match).

        Matches against section title and, for static sections, the body HTML
        (tags stripped).  Callable sections (extensions, keyboard_shortcuts)
        are matched on title only and kept visible on a body-only match.
        """
        needle = text.strip().lower()
        corpus: Dict[str, str] = getattr(self, '_help_search_corpus', {})
        for key, title, item in self._help_section_items:
            if not needle:
                item.setHidden(False)
                continue
            title_match = needle in title.lower()
            body_match = needle in corpus.get(key, '')
            item.setHidden(not (title_match or body_match))

        # If the current item is now hidden, select the first visible one.
        current = self._help_nav.currentItem()
        if current is not None and current.isHidden():
            for row in range(self._help_nav.count()):
                candidate = self._help_nav.item(row)
                if not candidate.isHidden():
                    self._help_nav.setCurrentItem(candidate)
                    break

    # ------------------------------------------------------------------
    # Live-generated section HTML
    # ------------------------------------------------------------------

    def _help_extensions_html(self) -> str:
        """
        Return HTML for the Extensions section.

        Prefers ALL discovered manifests (``_extensions_manifests``) so the
        reference is complete regardless of which extensions are enabled;
        falls back to loaded instances (``_extension_instances``).  Grouped
        by sidebar section.
        """
        manifests = getattr(self, '_extensions_manifests', None)
        instances = getattr(self, '_extension_instances', {})

        # Build list of (name, description, section).
        entries: List[Tuple[str, str, str]] = []
        if manifests:
            for _m in manifests:
                entries.append((
                    _m.name, getattr(_m, 'description', ''),
                    getattr(_m, 'sidebar_section', 'Extensions')))
        elif instances:
            for _ext_id, inst in instances.items():
                name = getattr(inst, 'NAME', None) or _ext_id
                desc = getattr(inst, 'DESCRIPTION', '')
                section = getattr(inst, 'SIDEBAR_SECTION', 'Extensions')
                entries.append((name, desc, section))

        # Group by section.
        grouped: dict = {}
        for name, desc, section in entries:
            grouped.setdefault(section, []).append((name, desc))

        rows = ['<h2>Extensions</h2>']
        rows.append(
            '<p>Extensions are optional feature packs in '
            '<code>Extensions/</code>. Enable them in '
            '<b>Settings &rarr; Extensions</b> and restart the Studio.</p>'
        )

        if not entries:
            rows.append('<p><em>No extensions are currently loaded.</em></p>')
        else:
            for section in sorted(grouped.keys()):
                rows.append(f'<h3>{section}</h3><ul>')
                for name, desc in grouped[section]:
                    rows.append(f'<li><b>{name}</b>')
                    if desc:
                        rows.append(f' &ndash; {desc}')
                    rows.append('</li>')
                rows.append('</ul>')

        rows.append(
            '<p><em>Built-in (not an extension):</em> '
            '<b>Gamepad</b> &ndash; USB gamepad Cartesian teleop '
            '(<code>gamepad_mixin.py</code>).</p>'
        )
        return '\n'.join(rows)

    def _help_shortcuts_html(self) -> str:
        """
        Return HTML for the Keyboard Shortcuts section.

        Assembles a table from: (a) fixed menubar shortcuts, (b) currently
        configured rebindable shortcuts from ``self.settings``, and
        (c) extension-nav shortcuts.
        """
        # (a) Fixed menubar shortcuts.
        menubar = [
            ('Ctrl+N', 'New Script (Script Editor)'),
            ('Ctrl+S', 'Save'),
            ('Ctrl+Q', 'Exit'),
            ('Ctrl+0', 'Navigate to Dashboard'),
            ('Ctrl+1', 'Navigate to Multi-Robot'),
            ('Ctrl+2', 'Navigate to Tuning Panel'),
            ('Ctrl+T', 'Toggle toolbar'),
            ('Ctrl+\\', 'Toggle sidebar'),
            ('Ctrl+L', 'Open log viewer'),
            ('Ctrl+M', 'Compact layout'),
            ('Ctrl+Shift+M', 'Minimal layout'),
            ('F11', 'Toggle fullscreen'),
            ('Ctrl+H', 'Home robot'),
            ('F5', 'Toggle simulation'),
            ('F6', 'Toggle FIC simulation'),
            ('F7', 'Start simulation + FIC'),
            ('Ctrl+Shift+Q', 'Stop all processes'),
            ('F8', 'Start hardware controller'),
            ('F9', 'Start hardware driver'),
            ('Ctrl+B', 'Compile / build'),
            ('F1', 'Open documentation'),
            ('Ctrl+/', 'Show shortcuts dialog'),
            ('Ctrl+,', 'Open Settings'),
        ]

        # (b) Rebindable shortcuts (live from settings).
        shortcuts_cfg = {}
        if hasattr(self, 'settings'):
            shortcuts_cfg = self.settings.get('shortcuts', {})

        _rebind_defaults = {
            'estop': ('Space', 'E-Stop'),
            'play_sequence': ('Ctrl+R', 'Play sequence'),
            'stop_sequence': ('Escape', 'Stop sequence'),
            'save_settings': ('Ctrl+S', 'Save settings'),
            'toggle_log': ('Ctrl+L', 'Toggle log viewer'),
            'panel_dashboard': ('Ctrl+0', 'Dashboard'),
            'panel_control': ('Ctrl+1', 'Multi-Robot'),
            'panel_tuning': ('Ctrl+3', 'Tuning Panel'),
            'panel_sequence': ('Ctrl+4', 'Sequences'),
            'panel_settings': ('Ctrl+5', 'Settings'),
        }
        rebindable = []
        for k, (default, label) in _rebind_defaults.items():
            binding = shortcuts_cfg.get(k, default)
            rebindable.append((binding, label))

        # (c) Extension-nav shortcuts.
        ext_nav = [
            ('Ctrl+3', 'Auto-Tuning page'),
            ('Ctrl+4', 'Sequences page'),
            ('Ctrl+5', 'Teach Mode page'),
            ('Ctrl+6', 'Script Editor page'),
            ('Ctrl+8', 'Experiments page'),
        ]

        def _rows(pairs: List[Tuple[str, str]]) -> str:
            lines = []
            for key, desc in pairs:
                lines.append(
                    f'<tr><td><code>{key}</code></td>'
                    f'<td>{desc}</td></tr>'
                )
            return '\n'.join(lines)

        html = (
            '<h2>Keyboard Shortcuts</h2>'
            '<h3>Menubar (non-rebindable)</h3>'
            '<table>'
            + _rows(menubar)
            + '</table>'
            '<h3>Rebindable (Settings &rarr; Shortcuts)</h3>'
            '<p>Current bindings shown; change in Settings &rarr; Shortcuts.</p>'
            '<table>'
            + _rows(rebindable)
            + '</table>'
            '<h3>Extension navigation</h3>'
            '<p>Note: Ctrl+3/4/5 conflict between rebindable and extension-nav '
            'slots; the last registered shortcut wins.</p>'
            '<table>'
            + _rows(ext_nav)
            + '</table>'
        )
        return html

    # ------------------------------------------------------------------
    # Themed HTML wrapper
    # ------------------------------------------------------------------

    def _help_themed_html(self, body: str) -> str:
        """
        Wrap ``body`` HTML in a ``<style>`` block from the current theme.

        Called each time a section is rendered or the theme changes so
        colours are always in sync with ``theme.current()``.
        """
        t = _theme.current()
        style = (
            'body {'
            f'background:{t["bg_base"]};'
            f'color:{t["text_primary"]};'
            'font-family:"Segoe UI",Arial,sans-serif;'
            'font-size:13px;'
            'line-height:1.55;'
            'margin:16px;'
            '}'
            'h2 {'
            f'color:{t["accent_blue"]};'
            'margin-top:0;'
            'margin-bottom:8px;'
            'font-size:18px;'
            '}'
            'h3 {'
            f'color:{t["accent_teal"]};'
            'margin-top:20px;'
            'margin-bottom:6px;'
            'font-size:14px;'
            '}'
            'p { margin:6px 0; }'
            'ul, ol { margin-left:20px; margin-bottom:8px; }'
            'li { margin:3px 0; }'
            'table { border-collapse:collapse; margin:8px 0; }'
            'td { padding:4px 16px 4px 0; vertical-align:top; }'
            'code {'
            f'background:{t["bg_overlay"]};'
            f'color:{t["accent_green"]};'
            'padding:1px 5px;'
            'border-radius:3px;'
            'font-family:Consolas,monospace;'
            'font-size:12px;'
            '}'
            'b {'
            f'color:{t["text_primary"]};'
            '}'
            'em {'
            f'color:{t["text_secondary"]};'
            '}'
        )
        return f'<html><head><style>{style}</style></head><body>{body}</body></html>'
