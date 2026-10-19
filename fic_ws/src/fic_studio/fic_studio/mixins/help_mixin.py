"""
Help page mixin for FIC Control Studio.

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

from fic_studio import version
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
<p><b>FIC Control Studio v{VERSION}</b> is the operator GUI for the Fractal
Impedance Control (FIC) framework. It runs the FIC C++ controller and the
simulator, talks to them over ROS 2 / DDS, and lets you drive, tune and
monitor a robot; up to four robot stations can work side by side.</p>
<p>These pages describe the Studio window (<code>fic_studio_pro --web-ui</code>,
the default desktop launcher). Keys named here are single keys: press them
when no text field has the focus.</p>

<h3>First-time setup</h3>
<ol>
  <li>The <b>First-Launch Wizard</b> opens on the very first start:
      <em>Welcome</em> (choose your robot), <em>Verify Workspace</em>,
      <em>Test Your Setup</em> (starts the simulator and the controller and
      checks that data arrives) and <em>Setup Complete</em>, which offers a
      first thing to do.</li>
  <li>Open <b>Settings</b> (sidebar, or Help &rarr; Settings) for the
      appearance, the network (ROS_DOMAIN_ID, partner IP), the simulator
      backend and the extensions.</li>
  <li>Choose a <b>Target</b> in the top bar (Simulation / Hardware / VR) and
      press the green <b>Start</b> (or <b>S</b>), then click it again to
      confirm. It starts what the target needs (simulator or driver, and the
      FIC controller) and turns into a red <b>Stop</b>.</li>
  <li>Open <b>Joint</b> (<b>J</b>) or <b>Cart</b> (<b>C</b>) on the control
      bar at the bottom, choose a mode, set a target and press
      <b>Send</b>.</li>
</ol>
<p>To start one part on its own, for example the simulator without the
controller, use <b>Settings &rarr; Processes</b>.</p>

<h3>Key concepts</h3>
<ul>
  <li><b>FIC controller</b> &ndash; a real-time C++ loop reading
      <code>config.xml</code>; parameters edited on the Tuning Panel are
      reloaded without a restart.</li>
  <li><b>Station</b> &ndash; the <em>Robot</em> selector in the top bar
      (Robot 1&ndash;4 or Master) picks which station this Studio is; each
      station uses its own <code>robot{N}_*</code> topics. Changing it
      restarts the Studio, so it is locked while running.</li>
  <li><b>Extensions</b> &ndash; optional pages, switched on in
      Settings &rarr; Extensions (restart to apply).</li>
  <li><b>E-STOP</b> &ndash; the red button at the top right,
      Robot &rarr; Emergency Stop or <b>Space</b>: halts all motion and
      stops the running processes. It is a software stop, not a substitute
      for the robot&rsquo;s physical stop, and it stays latched until
      <b>Reset</b> on the red banner.</li>
</ul>
""".replace('{VERSION}', version.RELEASE)

# -- 2. Window Layout ---------------------------------------------------------

_HTML_WINDOW_LAYOUT = """
<h2>Window Layout</h2>

<h3>Menu bar</h3>
<p>File, View, Robot, Simulation, Tools and Help. View shows or hides the
toolbar and the sidebar; Robot holds the control modes and the emergency
stop; Simulation starts and stops the simulator and the controller. The
<b>Theme</b> button on the right switches light and dark (<b>T</b>).</p>

<h3>Top bar</h3>
<table>
  <tr><td><b>Target</b></td>
      <td>Simulation / Hardware / VR: what Start runs.</td></tr>
  <tr><td><b>Start / Stop</b></td>
      <td>Green Start (click twice, or <b>S</b>) starts the target&rsquo;s
          processes; it then reads Stop, in red. The text beside it says
          Idle, Running or Partial.</td></tr>
  <tr><td><b>EXP</b></td>
      <td>Start / stop an experiment recording (Experiments extension).</td></tr>
  <tr><td><b>BAG</b></td>
      <td>Start / stop a rosbag recording.</td></tr>
  <tr><td><b>PLT</b></td>
      <td>Open the live plotter.</td></tr>
  <tr><td><b>VIEW</b></td>
      <td>Start the contact viewer.</td></tr>
  <tr><td><b>Robot</b></td>
      <td>Two selectors: the station (Robot 1&ndash;4, Master) and the
          robot model.</td></tr>
  <tr><td><b>E-STOP</b></td>
      <td>Emergency stop (also <b>Space</b>). <b>Esc</b> only closes
          menus and panels.</td></tr>
</table>
<p>EXP, BAG, PLT and VIEW ask for a second click to confirm.</p>

<h3>Sidebar</h3>
<p>The &#9776; button (top left) or View &rarr; Show Sidebar shows or hides
it; its Search box filters the pages. The main pages come first, then the
extension pages under their own headings, then System (Robot Config, Topic
Monitor, Safety); Users, Settings, Logs and Help sit at the bottom.
<b>[</b> and <b>]</b> go to the previous and next page.</p>

<h3>Control bar</h3>
<p>Docked at the bottom of every page: see <em>Pages &rarr; Control
bar</em>.</p>

<h3>Status bar</h3>
<table>
  <tr><td><b>ROS2</b></td><td>The Studio&rsquo;s ROS 2 node is up.</td></tr>
  <tr><td><b>FIC</b></td><td>The FIC controller is running.</td></tr>
  <tr><td><b>SIM</b></td><td>The simulator is running.</td></tr>
  <tr><td><b>BAGS</b></td><td>A rosbag is recording.</td></tr>
  <tr><td><b>INNER</b></td><td>The inner control loop in use.</td></tr>
  <tr><td><b>TELEOP</b></td><td>Teleoperation role (leader / follower).</td></tr>
  <tr><td><b>GAMEPAD</b></td><td>A gamepad is connected.</td></tr>
  <tr><td><b>WEB</b></td><td>Web Remote clients connected.</td></tr>
  <tr><td><b>STUDIO</b></td><td>(right) The link to the Studio: rate, stale or offline.</td></tr>
</table>
<p>Colours: green = ok / live, orange = warning or active, red = error or
offline, grey = off. Messages go to the <b>Logs</b> page (View &rarr; Logs),
filtered by level, source and text.</p>
"""

# -- 3. Pages -----------------------------------------------------------------

_HTML_PAGES = """
<h2>Pages</h2>

<h3>Dashboard</h3>
<p>The live view at the top: the robot in 3-D with path overlays (trail,
commanded, target, zones, force) beside live plots of the end effector,
the joints and the joint efforts. Below it, two tabs refreshed about 10
times a second; a channel with no live source shows a dash:</p>
<ul>
  <li><b>Overview</b> &ndash; <b>Base</b> (control mode, target, profile,
      telemetry rate); <b>Joints</b> (commanded, position, error, velocity,
      torque); <b>End effector</b> (pose and error as x y z and ox oy oz, and
      the tool wrench with a contact indicator); <b>Safety</b> (joint,
      velocity and force/torque limits).</li>
  <li><b>Detailed</b> &ndash; per-joint limits and headroom, the whole pose,
      twist and wrench, the robot&rsquo;s model data and the telemetry
      channels.</li>
</ul>

<h3>Control bar (bottom of every page)</h3>
<ul>
  <li><b>Joint</b> (<b>J</b>) and <b>Cart</b> (<b>C</b>) &ndash; each opens
      a panel with its four modes (Impedance, Imp.&nbsp;+&nbsp;Planner,
      Admittance, HAP-FIC; <b>1</b>&ndash;<b>4</b>), its switches, the
      targets (click to type or drag to scrub), Live publish, Reset, Send,
      preset poses (Cart) and, on modes 7 and 8, a desired wrench.</li>
  <li><b>Gripper</b> (<b>G</b>) &ndash; open, close or a width; hidden for
      robots without one.</li>
  <li><b>Home</b> (<b>Shift+H</b>) &ndash; the home joint angles in joint
      modes, the default end-effector target in Cartesian modes.</li>
  <li><b>Contact</b> &ndash; calibrate the contact estimate and compare
      with ground truth.</li>
  <li><b>Snapshot</b> and <b>Quick Actions</b> (Sequences extension) &ndash;
      Snapshot adds the current pose to a sequence; Quick Actions plays,
      steps or loops a sequence, sets its speed, and runs a line, circle or
      figure-eight from where the arm is.</li>
  <li>On the right: the mode, the target (SIM / HW / VR) and the loop
      rate.</li>
</ul>
<p>With the VR target the headset drives, so the command fields are
off.</p>

<h3>Multi-Robot</h3>
<ul>
  <li><b>Robots</b> &ndash; a card per station (Control, Pause, Stop,
      Remove) and <b>+ Add robot</b>.</li>
  <li><b>Studios</b> &ndash; the other Studios on the network: drive, start
      the simulation or the hardware, stop; <b>Stop every robot</b> and
      <b>Pause all</b>.</li>
  <li><b>Teleoperation</b> &ndash; leader, follower, scale and maximum speed,
      then <b>Start teleop</b>.</li>
</ul>

<h3>Locomotion</h3>
<p>Gaits and velocity commands; shown for legged robots only.</p>

<h3>Tuning Panel</h3>
<ul>
  <li>Mode, joint and config (<b>+ Config</b>, <b>Make active</b>).</li>
  <li>The force profile: drag the handles, scroll to change K<sub>0</sub>;
      the curve follows your edits before you save.</li>
  <li>Controller, Planner and Admittance parameters; a live strip of
      tracking error, effort and wrench.</li>
  <li><b>Live trial</b> tries edits on the running controller (Keep or
      Discard). <b>Save to XML</b> writes <code>config.xml</code>, which the
      controller reloads without a restart; <b>Load from XML</b> and
      <b>View XML</b>.</li>
</ul>

<h3>Robot Config</h3>
<p>Every robot the Studio can run. Drop a URDF, a .xacro, a
description-package folder or a .zip anywhere on the page (up to 200&nbsp;MB)
to add one, or start a blank robot. Each robot has five tabs: Overview
(what is missing before it can be simulated), Joints, Frames, Hardware and
Safety.</p>

<h3>Topic Monitor</h3>
<p>Live ROS 2 topics with their message types and values.</p>

<h3>Safety</h3>
<p>Cartesian zones, joint limits, velocity limits and force/torque limits;
edits are reviewed, then applied to every robot&rsquo;s controller.</p>

<h3>Users</h3>
<p>Web Remote accounts, their pages and whether they may use the hardware
(admins only).</p>

<h3>Settings</h3>
<p><b>Processes</b> comes first and stays open: start the simulator
without the controller, the FIC controller or the robot driver on its own,
and stop them. The other sections start folded (Expand all opens them):
Appearance, Startup, Network (restart to apply), Simulation, Playback
tolerances, Shortcuts, Extensions (restart to apply), Gamepad, Web Remote
and Developer (with the build buttons). On a wide window they flow into
columns.</p>

<h3>Logs</h3>
<p>Every message, filtered by level, source and text.</p>

<h3>Extension pages</h3>
<p>One page per enabled extension, under its own heading in the sidebar
(for example the MuJoCo Playground under Programming): see
<em>Extensions</em>.</p>
"""

# -- 4. Control Modes ---------------------------------------------------------

_HTML_CONTROL_MODES = """
<h2>Control Modes</h2>
<p>Choose a mode in the control bar&rsquo;s <b>Joint</b> or <b>Cart</b>
panel (then <b>1</b>&ndash;<b>4</b>), or Robot &rarr; Control Mode. Joint
modes show joint rows; Cartesian modes show the end-effector pose.</p>

<table>
  <tr><td><b>1 &mdash; Joint Impedance</b></td>
      <td>Position stiffness K<sub>0</sub>(q<sub>des</sub>&minus;q<sub>act</sub>),
          no damping.</td></tr>
  <tr><td><b>2 &mdash; Joint Impedance + Planner</b></td>
      <td>Smooth trajectory (damping ratio, desired velocity, maximum
          acceleration). The usual mode for joint-space driving.</td></tr>
  <tr><td><b>3 &mdash; Joint Admittance</b></td>
      <td>Yields to external torque (inverse mass, damping, flow, pose
          range).</td></tr>
  <tr><td><b>4 &mdash; Joint HAP-FIC</b></td>
      <td>Joint admittance bounded by each joint&rsquo;s FIC law: yields to a
          push and returns on release.</td></tr>
  <tr><td><b>5 &mdash; Cartesian Impedance</b></td>
      <td>End-effector spring, no damping.</td></tr>
  <tr><td><b>6 &mdash; Cartesian Impedance with Planner</b></td>
      <td>Smooth end-effector trajectory; the usual mode for Cartesian
          driving.</td></tr>
  <tr><td><b>7 &mdash; Cartesian Admittance</b></td>
      <td>The end effector yields to the external wrench and stays where it
          is pushed, within the admittance pose range (by default
          &plusmn;0.3&nbsp;m and &plusmn;0.5&nbsp;rad; Tuning Panel &rarr;
          Admittance).</td></tr>
  <tr><td><b>8 &mdash; Cartesian HAP-FIC</b></td>
      <td>Cartesian admittance bounded by the FIC law: yields to a push and
          returns on release.</td></tr>
</table>
<p>In modes 1 and 2 a target far from the arm is approached at no more than
the <b>joint target speed</b> (Safety page; 0 = off), so a distant Home or
preset does not hit the velocity limits.</p>

<h3>Switches (next to the mode buttons)</h3>
<table>
  <tr><td><b>Postural</b> (<b>P</b>, Cart)</td>
      <td>Null-space joint-target overlay.</td></tr>
  <tr><td><b>Elbow</b> (<b>E</b>, Cart)</td>
      <td>Null-space elbow-hold overlay.</td></tr>
  <tr><td><b>H-PMP</b> (<b>F</b>, Cart)</td>
      <td>The Cartesian controller nested in the joint HAP-FIC, on modes
          5&ndash;8 (on 7 and 8 the mode&rsquo;s task-space admittance also
          moves its goal).</td></tr>
  <tr><td><b>Planner</b> (<b>P</b>, Joint)</td>
      <td>The planner inside the admittance modes 3 and 4.</td></tr>
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

<p>Cart Step is applied every 10&nbsp;ms (100&nbsp;Hz), so Medium at full stick
moves the end effector at 0.1&nbsp;m/s. <b>A</b> (Home) switches to Joint
Impedance and needs the robot&rsquo;s <code>q_home_deg</code>; <b>B</b> runs the
Studio&rsquo;s full E-STOP. On a legged robot the sticks drive the body velocity
(<code>{rid}_cmd_vel</code>) instead.</p>

<h3>Configuration</h3>
<p>The <b>Gamepad</b> section of the Settings page: deadzone, axis inversion,
the button map (every button can be reassigned) and, with more than one pad,
which robot each pad drives. A pad plugged in while the Studio runs is found
within half a second. The status bar shows the current state (e.g.
<code>Joint J1,J2 Med | Impedance</code>).</p>

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
<p>The green <b>Start</b> in the top bar (click twice, or <b>S</b>) starts what
the target needs; the red <b>Stop</b> stops it:</p>
<table>
  <tr><td><b>Simulation</b></td><td>The simulator and the FIC simulation controller.</td></tr>
  <tr><td><b>Hardware</b></td><td>The robot driver and the FIC hardware controller.
      Refused when the robot has no <code>driver_command</code> (unless an
      inner-loop MCU is present).</td></tr>
  <tr><td><b>VR</b></td><td>The VR relay and one FIC controller per VR robot.</td></tr>
</table>
<p>One process on its own: <b>Settings &rarr; Processes</b> (the simulator
without the controller, either controller, the robot driver), or
<b>Simulation &rarr; Processes&hellip;</b>, which lists them all:</p>
<table>
  <tr><td><b>Simulation</b></td>
      <td>The physics simulator: MuJoCo, or Newton when built (Settings &rarr;
          Simulation; a robot&rsquo;s own <code>sim_backend</code> wins). A gripper
          bridge starts with it.</td></tr>
  <tr><td><b>FIC controller (simulation)</b></td>
      <td>The C++ FIC loop on the simulator; reads <code>config.xml</code>.</td></tr>
  <tr><td><b>Robot driver</b></td>
      <td>The robot&rsquo;s own driver (Kinova Kortex, Franka FCI, &hellip;).</td></tr>
  <tr><td><b>FIC controller (hardware)</b></td>
      <td>The C++ FIC loop on the real robot.</td></tr>
  <tr><td><b>Locomotion</b></td><td>Legged robots only.</td></tr>
  <tr><td><b>VR relay</b></td><td>The ROS node that relays the VR headset.</td></tr>
  <tr><td><b>Bag recording</b></td>
      <td>A rosbag of the session (<b>BAG</b>, or <b>EXP</b> for a named
          experiment), saved in <code>&lt;experiments&gt;/bags/</code>.</td></tr>
</table>
<p><b>PLT</b> opens the live plotter and <b>VIEW</b> the contact viewer. The
status bar shows what runs (FIC, SIM, BAGS, &hellip;). Process output is
logged to <code>/tmp/fic_sim_*.log</code>, <code>/tmp/fic_bag_*.log</code> and
<code>/tmp/fic_vr_relay_*.log</code>.</p>
"""

# -- 9. Safety ----------------------------------------------------------------

_HTML_SAFETY = """
<h2>Safety</h2>

<h3>E-STOP</h3>
<p>Triggered by the <b>E-STOP</b> button (top right), Robot &rarr; Emergency
Stop, <b>Space</b> or gamepad <b>B</b> (<b>Esc</b> never fires it). It is
a software stop: it cannot replace the robot&rsquo;s own emergency stop,
and it cannot act when the Studio is not answering (the button then turns
grey and says so). It:</p>
<ol>
  <li>tells every loaded extension to halt, and ends teleoperation (the
      partner arms are stopped too);</li>
  <li>commands a graceful torque ramp-down (&ldquo;Stop Controller&rdquo;) to
      every robot and waits 250&nbsp;ms, so the robot settles into a hold
      instead of keeping its last torque;</li>
  <li>holds each robot at its last joint pose;</li>
  <li>stops every process this Studio started (controllers, driver,
      simulator, rosbag, plotter). Other users' simulators and launches on
      the same computer keep running;</li>
  <li>latches: a red banner says when and by whom (the account, the
      gamepad or the station that sent it), the tab title starts with
      &ldquo;E-STOP&rdquo;, and Start, processes, motion, Home and
      teleoperation are refused until <b>Reset&hellip;</b> on the banner.
      Reset asks you to check the robot and starts nothing.</li>
</ol>
<p>Every E-STOP, reset, session or process start and stop, target change,
teleoperation request and control hand-over is also written to
<code>~/.fic_studio/audit.log</code> (one JSON line each, with the account
name; it cannot be cleared from the UI).</p>

<h3>Safety page</h3>
<p>Safety zones, the workspace box (x, y, z minimum and maximum in metres,
checked at the tool), joint limits (degrees) and velocity limits, each with a
live readout of where the robot is. Edits are reviewed, then applied to every
robot&rsquo;s controller and saved with the robot selected in the toolbar.</p>
<p>The toolbar on <em>Zones in 3D</em> makes and moves zones, all reviewed and
applied the same way:</p>
<ul>
  <li><b>Select and turn</b> (arrow): click a zone to show its handles and
      drag them to move, lift, resize or turn it; drag a face of the workspace
      box along its axis; drag elsewhere to turn the view.</li>
  <li><b>Box, Sphere, Cylinder</b>: click the floor where the shape goes.</li>
  <li><b>Draw an outline</b> (pen): click the floor at each corner; click the
      first corner, double-click or press Enter to close it, and set its bottom
      and top. Backspace undoes a corner, Esc cancels.</li>
  <li><b>Bin</b>: deletes the selected zone (Review and Apply to remove it
      from the controllers).</li>
  <li><b>Keep out / Keep in</b>: what new zones are. A zone is checked along
      the whole arm (from the joint set under <em>The arm checked</em> to the
      tool), not only at the tool point.</li>
  <li><b>Table here, Wall here, Set here</b>: guide the arm by hand (joint
      admittance) until the tool rests on the table or wall and press one; the
      workspace limit goes at the tool&rsquo;s position, the margin (20&nbsp;mm
      by default) inside. Move the arm clear before applying.</li>
</ul>
<p>Every value also has a slider next to its number. When a robot touches a
zone or a workspace face, or goes past it, a warning drops down and stays in
<b>Notices</b>.</p>
<ul>
  <li>The controller <b>clamps commanded targets</b> to the enabled joint and
      Cartesian limits and holds the end effector inside the zone with a
      virtual wall.</li>
  <li><b>Joint modes and the workspace:</b> a joint target whose end effector
      would leave the enabled Cartesian box is rejected, and one that would take
      the arm further into a safety zone; a link past a zone is pushed back by a
      virtual wall in every mode.</li>
  <li>The Studio checks the position 10 times a second, warns in the outer
      10&nbsp;% of each range and logs every violation.</li>
  <li><b>Stop on violation</b> (on by default) is a backstop for when
      something pushes the arm through a limit: a robot more than 30&nbsp;mm
      past a Cartesian limit or a safety zone, or 2&deg; past a joint limit, gets the graceful
      stop (E-STOP&rsquo;s ramp-down and pose hold, without stopping any
      process). Closer than that the controller holds it at the limit by
      itself, so an arm resting against a limit is not stopped. Only that
      robot is stopped, once: it can be stopped again after it has come back
      within half that distance, so you can restart the controller and move
      it back in.</li>
  <li><b>Velocity limits:</b> the maximum joint velocity acts as damping on
      every joint (simulation uses the robot&rsquo;s <code>sim_max_joint_vel</code>
      unless &ldquo;simulate like the real robot&rdquo; is on). The maximum
      Cartesian velocity is stored but not yet used by the controllers.</li>
  <li><b>Joint target speed</b> (rad/s; 0 = off): in modes 1 and 2 the joint
      target moves towards a distant command at no more than this speed, so
      Home or a preset far from the arm does not hit the velocity limits.
      The Kinova uses 0.5&nbsp;rad/s on the real arm; simulation starts with it
      off, and Apply sends the value shown.</li>
</ul>
<p>Limit topics (per station N, <code>Float64MultiArray</code>):
<code>robot{N}_safety/cartesian_limits</code>,
<code>robot{N}_safety/joint_limits</code>,
<code>robot{N}_safety/velocity_limits</code> (joint velocity, Cartesian
velocity, torque scale, joint target speed).</p>

<h3>Output-torque guard</h3>
<p>Before publishing, commanded torques are forced finite (NaN/Inf&nbsp;&rarr;&nbsp;0)
and clamped to a safe per-joint ceiling &mdash; a backstop against control
output that has gone non-finite or runaway.</p>

<h3>Teleoperation</h3>
<p>Pair a leader and a follower on <b>Multi-Robot &rarr; Teleoperation</b>.
Bilateral force reflection to the operator&rsquo;s haptic device is
<b>gain-scaled, NaN-guarded and saturated</b> (force and torque ceilings), so
a spiking F/T reading cannot be amplified onto the operator.</p>
"""

# -- 10. Troubleshooting ------------------------------------------------------

_HTML_TROUBLESHOOTING = """
<h2>Troubleshooting</h2>

<h3>The robot does not respond to Send</h3>
<ul>
  <li>Check the <b>FIC</b> LED: the controller must be running.</li>
  <li>Check that <b>ROS_DOMAIN_ID</b> matches the controller (Settings &rarr;
      Network, then restart). A <code>ROS_DOMAIN_ID</code> already set in the
      environment, e.g. in <code>~/.bashrc</code>, wins over the setting.</li>
  <li>Check the station in the top bar&rsquo;s <em>Robot</em> selector.</li>
  <li>On Hardware, check the <b>ROS2</b> LED and Simulation &rarr;
      Processes&hellip; for the driver and the hardware controller.</li>
  <li>Within 2&nbsp;s of an E-STOP, Start is refused: wait and try again.</li>
</ul>

<h3>Hardware Start is refused</h3>
<ul>
  <li>&ldquo;No driver command and no inner-loop MCU&rdquo;: set the
      robot&rsquo;s driver on <b>Robot Config &rarr; Hardware</b>.</li>
</ul>

<h3>The simulator does not start</h3>
<ul>
  <li>Read <code>/tmp/fic_sim_*.log</code> and the <b>Logs</b> page.</li>
  <li>Try the other backend in Settings &rarr; Simulation (MuJoCo or
      Newton).</li>
  <li>Check the workspace with the setup wizard (Help &rarr; Re-run Setup
      Wizard&hellip; in the desktop window).</li>
</ul>

<h3>The gamepad is not detected</h3>
<ul>
  <li>Check that <code>/dev/input/jsN</code> exists
      (<code>ls /dev/input/js*</code>); the Studio looks for pads twice a
      second.</li>
  <li>&ldquo;Failed to open&rdquo; in Logs: your user cannot read
      <code>/dev/input</code> (add it to the <code>input</code> group).</li>
  <li>The <b>GAMEPAD</b> LED turns green when the pad is found.</li>
</ul>

<h3>An extension page is missing</h3>
<ul>
  <li>Switch it on in Settings &rarr; Extensions, then <b>restart the
      Studio</b> (extensions load at start).</li>
  <li>Look for import errors on the Logs page.</li>
</ul>

<h3>Tuning changes are not applied</h3>
<ul>
  <li>Press <b>Save to XML</b> on the Tuning Panel: the Studio then tells
      the controller to re-read <code>config.xml</code>. A change made to the
      file by hand is read only after a profile change or a restart of the
      controller.</li>
</ul>

<h3>Dropping a file does nothing</h3>
<ul>
  <li>Drops work in the Studio window only, not in a remote browser.</li>
  <li>MuJoCo Playground: STL and OBJ, and PLY, OFF, GLB and 3MF when
      <code>trimesh</code> is installed; up to 250&nbsp;MB; a mesh over
      200,000 triangles is simplified. Robot Config: up to 200&nbsp;MB.</li>
</ul>

<h3>The theme looks wrong after a restart</h3>
<ul>
  <li>Settings &rarr; Appearance (palette and brightness), then <b>Save
      settings</b>. The Theme button (<b>T</b>) switches light and dark at
      once.</li>
</ul>
"""

# -- 11. Glossary -------------------------------------------------------------

_HTML_GLOSSARY = """
<h2>Glossary</h2>
<table>
  <tr><td><b>FIC</b></td>
      <td>Fractal Impedance Control &mdash; the real-time controller at the
          core of the framework (MIT licence).</td></tr>
  <tr><td><b>EE</b></td>
      <td>End-Effector &mdash; the robot&rsquo;s tool / flange point.</td></tr>
  <tr><td><b>DOF</b></td>
      <td>Degrees of Freedom.</td></tr>
  <tr><td><b>DDS</b></td>
      <td>Data Distribution Service &mdash; the ROS 2 middleware layer (the
          ROS default, Fast DDS on Jazzy, unless <code>RMW_IMPLEMENTATION</code>
          says otherwise).</td></tr>
  <tr><td><b>ROS 2</b></td>
      <td>Robot Operating System 2 &mdash; the inter-process communication
          layer used by the Studio.</td></tr>
  <tr><td><b>config.xml</b></td>
      <td>The FIC controller&rsquo;s parameter file; edited on the Tuning
          Panel, and re-read by the running controller on Save to XML.</td></tr>
  <tr><td><b>Admittance</b></td>
      <td>A compliance mode where the robot yields to applied forces /
          torques (modes 3, 4, 7, 8).</td></tr>
  <tr><td><b>Impedance</b></td>
      <td>A stiffness mode where the robot resists deviation from the
          commanded position (modes 1, 2, 5, 6).</td></tr>
  <tr><td><b>H-PMP</b></td>
      <td>The Cartesian controller nested in the joint HAP-FIC (mode 4's law on every joint), on
          by default in modes 5&ndash;8: on 5 and 6 the arm returns to the commanded goal after a
          push, on 7 the goal stays where it was pushed, on 8 it gives way up to 5&nbsp;cm. Turned
          off with the H-PMP switch (<b>F</b>) on the Cart panel (the Cartesian torques then go
          directly to the joints).</td></tr>
  <tr><td><b>Null-space</b></td>
      <td>The set of joint motions that do not move the EE; used by the
          Postural and Elbow overlays.</td></tr>
  <tr><td><b>Station</b></td>
      <td>A numbered robot slot (1&ndash;4), or the Master Studio; each
          station maps to a <code>robot{N}_*</code> topic namespace.</td></tr>
  <tr><td><b>Extension</b></td>
      <td>An optional feature pack living in <code>Extensions/</code>;
          enabled in Settings &rarr; Extensions and loaded at startup.</td></tr>
  <tr><td><b>Leader / Follower</b></td>
      <td>Teleoperation roles: the leader is moved by the operator and
          receives force feedback; the follower follows it.</td></tr>
  <tr><td><b>Master</b></td>
      <td>A Studio station that oversees the others (Multi-Robot).</td></tr>
  <tr><td><b>MuJoCo</b></td>
      <td>Physics simulator backend (CPU); alternative to Newton (GPU).</td></tr>
  <tr><td><b>Newton</b></td>
      <td>NVIDIA Warp-based GPU physics backend (optional install).</td></tr>
</table>
"""

# -- 12. About ----------------------------------------------------------------

_HTML_ABOUT = """
<h2>About FIC Control Studio</h2>
<p><b>FIC Control Studio v{VERSION}</b></p>
<p>The operator GUI for the Fractal Impedance Control robot framework.</p>
<ul>
  <li><b>Package:</b> <code>fic_studio</code></li>
  <li><b>Settings file:</b> <code>~/.robot_selector_gui_settings.yaml</code></li>
  <li><b>Tech stack:</b> Python 3, PyQt5 and QtWebEngine (the window is a web
      page, <code>web_remote/ui</code>, also served to remote browsers), ROS 2
      Jazzy, MuJoCo 3.15 / Newton 1.7</li>
  <li><b>Licence:</b> MIT</li>
  <li><b>Authors:</b> Adnan Takriti and Carlo Tiseo, University of Sussex</li>
  <li><b>Source:</b> <code>https://github.com/TRL-Sussex/FIC_Interface</code></li>
</ul>
<p>Optional features live in <code>Extensions/&lt;name&gt;/</code> and are
loaded at start.</p>
<p>The full documentation: <b>Open the documentation</b> on this page, or
Help &rarr; Open Documentation (web).</p>
""".replace('{VERSION}', version.RELEASE)


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
        '<p>The <b>Script Editor</b> extension (sidebar &rarr; Programming, or '
        'File &rarr; New Script) lets you write and run '
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
        'hpmp(), admittance_planner().</li>'
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

            def publish_hpmp_enable(self, *_a, **_kw):
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
            '<b>Settings &rarr; Extensions</b> and restart the Studio. Each one '
            'gets a page in the sidebar under its own heading. Extensions with '
            'their own web page show it; the others show their controls as a '
            'list, with any plot or 3-D view kept to the desktop window.</p>'
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
            '<b>Gamepad</b> &ndash; USB gamepad teleoperation in joint and '
            'Cartesian space, and body velocity for legged robots '
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
            ('Ctrl+1', 'Navigate to Robot Control'),
            ('Ctrl+3', 'Navigate to Tuning Panel'),
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
            'panel_control': ('Ctrl+1', 'Robot Control'),
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

        # (d) The Studio window's own keys (web_remote/ui/shell.js onKey): single keys, no Ctrl.
        window_keys = [
            ('Space', 'E-STOP (not while a button or a text field has the focus)'),
            ('Esc', 'Cancel a pending confirm, close a menu, panel or dialog, '
                    'or leave a text field; with nothing to close, E-STOP'),
            ('J / C', 'Joint / Cartesian control (first mode)'),
            ('1 &ndash; 4', 'Mode within Joint or Cartesian'),
            ('F / P / E', 'H-PMP / Postural (Cart) or Planner (Joint) / Elbow'),
            ('G', 'Gripper'),
            ('Shift+H', 'Home'),
            ('S', 'Start / Stop (press again to confirm)'),
            ('T', 'Theme: light / dark'),
            ('[ / ]', 'Previous / next page'),
            ('Shift+&uarr; / &darr;', '1&deg; steps in the control bar&rsquo;s number fields'),
        ]

        html = (
            '<h2>Keyboard Shortcuts</h2>'
            '<h3>Studio window</h3>'
            '<p>Single keys, used when no text field has the focus.</p>'
            '<table>'
            + _rows(window_keys)
            + '</table>'
            '<p>The tables below are for the classic desktop window '
            '(<code>fic_studio_pro</code> without <code>--web-ui</code>); '
            'the Studio window ignores Ctrl combinations.</p>'
            '<h3>Classic window: menu bar</h3>'
            '<table>'
            + _rows(menubar)
            + '</table>'
            '<h3>Classic window: rebindable (Settings &rarr; Shortcuts)</h3>'
            '<p>Current bindings shown; change in Settings &rarr; Shortcuts.</p>'
            '<table>'
            + _rows(rebindable)
            + '</table>'
            '<h3>Classic window: extension pages</h3>'
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
